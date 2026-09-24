#!/usr/bin/env bash
# Family compile check for the CH32V203 stratum (part of every driver's
# definition of done, the twin of cli/checks/check_ch32v00x.sh).
#
# Positive: every test/family_ch32vx03/*.cpp must COMPILE for every part
# in PARTS with the project's own flags - the part's definition
# (CH32V203C8 and the like, what device.hpp asks for its table) and the
# part's own ISA and ABI - or, when the TU carries a "// mcu: <list>"
# line, for those parts alone: a TU that names a USART, a pad or a block
# some package has not got says so there. Thirteen parts, two ISAs: the
# nine CH32V203 are the QingKe V4B (rv32imac_xw, ilp32) and the four
# CH32V303 the V4F (rv32imafc_xw, ilp32f - the same core with a
# single-precision FPU), each compiled with the pair the part table
# (ch32vx03/cmake/ch32vx03-parts.cmake) gives it.
# Every positive is compiled BOTH WAYS the project can build an image:
# with the core's hardware prologue/epilogue (-DBRIO_CH32_HPE=1, the
# CH32VX03_HPE option, WCH's interrupt attribute) and without it (gcc's
# own prologue), since pfic.hpp's BRIO_CH32_INTERRUPT is one spelling
# with two expansions and a fixture that binds a vector proves both.
# Negative: every test/family_ch32vx03/neg/*.cpp must FAIL to compile
# for each part named on its "// mcu: <list>" line.
#
# No CMake coupling on purpose (same as the other six scripts): the
# compiler is called directly, the sweep takes seconds, no hardware.
#
# Usage: brio check ch32vx03            all TUs, all parts
#        brio check ch32vx03 pin        only TUs/negatives matching "pin"
set -u
cd "$(dirname "$0")/../.."

CXX=/sw/wch-riscv/bin/riscv32-wch-elf-g++
COMMON="-std=gnu++23 -Os -Wall -Wextra -Werror -fno-exceptions -fno-rtti -c -Ibrio"
V4B="-march=rv32imac_xw -mabi=ilp32"
V4F="-march=rv32imafc_xw -mabi=ilp32f"
PARTS="ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
       ch32v303cb ch32v303rb ch32v303rc ch32v303vc"
FILTER="${1:-}"
fail=0

# The part's own flags: the part DEFINITION, which is what device.hpp
# asks for its table, and the ISA and ABI - the three things
# ch32vx03/CMakeLists.txt derives from CH32VX03_MCU that the compiler
# sees (cmake/ch32vx03-parts.cmake is the table).
part_flags() {
    case "$1" in
        ch32v203f6) echo "-DCH32V203F6 $V4B $COMMON" ;;
        ch32v203f8) echo "-DCH32V203F8 $V4B $COMMON" ;;
        ch32v203g6) echo "-DCH32V203G6 $V4B $COMMON" ;;
        ch32v203g8) echo "-DCH32V203G8 $V4B $COMMON" ;;
        ch32v203k6) echo "-DCH32V203K6 $V4B $COMMON" ;;
        ch32v203k8) echo "-DCH32V203K8 $V4B $COMMON" ;;
        ch32v203c6) echo "-DCH32V203C6 $V4B $COMMON" ;;
        ch32v203c8) echo "-DCH32V203C8 $V4B $COMMON" ;;
        ch32v203rb) echo "-DCH32V203RB $V4B $COMMON" ;;
        ch32v303cb) echo "-DCH32V303CB $V4F $COMMON" ;;
        ch32v303rb) echo "-DCH32V303RB $V4F $COMMON" ;;
        ch32v303rc) echo "-DCH32V303RC $V4F $COMMON" ;;
        ch32v303vc) echo "-DCH32V303VC $V4F $COMMON" ;;
        *) echo "unknown part $1" >&2; exit 2 ;;
    esac
}

for tu in test/family_ch32vx03/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    parts="$(sed -n 's|^// mcu:||p' "$tu")"
    [ -n "$parts" ] || parts="$PARTS"
    for part in $parts; do
        for hpe in 0 1; do
            if $CXX $(part_flags "$part") -DBRIO_CH32_HPE=$hpe "$tu" -o /dev/null 2>/tmp/check_ch32vx03_err; then
                line="$line $part/hpe$hpe"
            else
                line="$line $part/hpe$hpe:FAIL"
                fail=1
                sed "s/^/    /" /tmp/check_ch32vx03_err | head -15
            fi
        done
    done
    echo "POS $line"
done

for tu in test/family_ch32vx03/neg/*.cpp; do
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

[ "$fail" -eq 0 ] && echo "check_ch32vx03: OK" || echo "check_ch32vx03: FAILURES"
exit "$fail"
