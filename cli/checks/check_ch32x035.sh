#!/usr/bin/env bash
# Family compile check for the CH32X035 stratum (part of every driver's
# definition of done, the twin of cli/checks/check_ch32vx03.sh).
#
# Positive: every test/family_ch32x035/*.cpp must COMPILE for every part
# in PARTS with the project's own flags - the part's definition
# (CH32X035F8 and the like, what device.hpp asks for its table) and the
# part's ISA and ABI - or, when the TU carries a "// mcu: <list>" line,
# for those parts alone: a TU that names a USART, a pad or a block some
# package has not got says so there. Seven parts, one ISA: every part of
# the series is the QingKe V4C (rv32imac_xw, ilp32), as the part table
# (ch32x035/cmake/ch32x035-parts.cmake) states per part.
# Every positive is compiled BOTH WAYS the project can build an image:
# with the core's hardware prologue/epilogue (-DBRIO_CH32_HPE=1, the
# CH32X035_HPE option, WCH's interrupt attribute) and without it (gcc's
# own prologue), since pfic.hpp's BRIO_CH32_INTERRUPT is one spelling with
# two expansions and a fixture that binds a vector proves both.
# Negative: every test/family_ch32x035/neg/*.cpp must FAIL to compile for
# each part named on its "// mcu: <list>" line.
#
# No CMake coupling on purpose (same as the other scripts): the compiler
# is called directly, the sweep takes seconds, no hardware.
#
# Usage: brio check ch32x035            all TUs, all parts
#        brio check ch32x035 pin        only TUs/negatives matching "pin"
set -u
cd "$(dirname "$0")/../.."

CXX=/sw/wch-riscv/bin/riscv32-wch-elf-g++
# A compiler that is not installed must not pass for a refusal: a command
# that cannot start fails like a TU that failed to compile, so every
# negative would read "refused" and every positive would fail for a reason
# that is not the code's. Say so and stop.
[ -x "$CXX" ] || { echo "$(basename "$0"): the compiler $CXX is not installed - nothing checked" >&2; exit 2; }
COMMON="-std=gnu++23 -Os -Wall -Wextra -Werror -fno-exceptions -fno-rtti -c -Ibrio"
V4C="-march=rv32imac_xw -mabi=ilp32"
PARTS="ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8"
FILTER="${1:-}"
fail=0

# The part's own flags: the part DEFINITION, which is what device.hpp asks
# for its table, and the ISA and ABI - the three things
# ch32x035/CMakeLists.txt derives from CH32X035_MCU that the compiler sees
# (cmake/ch32x035-parts.cmake is the table).
part_flags() {
    case "$1" in
        ch32x035r8)  echo "-DCH32X035R8 $V4C $COMMON" ;;
        ch32x035c8)  echo "-DCH32X035C8 $V4C $COMMON" ;;
        ch32x035g8u) echo "-DCH32X035G8U $V4C $COMMON" ;;
        ch32x035g8r) echo "-DCH32X035G8R $V4C $COMMON" ;;
        ch32x035f8)  echo "-DCH32X035F8 $V4C $COMMON" ;;
        ch32x035f7)  echo "-DCH32X035F7 $V4C $COMMON" ;;
        ch32x033f8)  echo "-DCH32X033F8 $V4C $COMMON" ;;
        *) echo "unknown part $1" >&2; exit 2 ;;
    esac
}

for tu in test/family_ch32x035/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    parts="$(sed -n 's|^// mcu:||p' "$tu")"
    [ -n "$parts" ] || parts="$PARTS"
    for part in $parts; do
        for hpe in 0 1; do
            if $CXX $(part_flags "$part") -DBRIO_CH32_HPE=$hpe "$tu" -o /dev/null 2>/tmp/check_ch32x035_err; then
                line="$line $part/hpe$hpe"
            else
                line="$line $part/hpe$hpe:FAIL"
                fail=1
                sed "s/^/    /" /tmp/check_ch32x035_err | head -15
            fi
        done
    done
    echo "POS $line"
done

for tu in test/family_ch32x035/neg/*.cpp; do
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

[ "$fail" -eq 0 ] && echo "check_ch32x035: OK" || echo "check_ch32x035: FAILURES"
exit "$fail"
