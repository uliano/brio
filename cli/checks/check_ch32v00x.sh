#!/usr/bin/env bash
# Family compile check for the CH32V00x stratum (part of every driver's
# definition of done, the RISC-V twin of cli/checks/check_stm32g0.sh).
#
# Positive: every test/family_ch32v00x/*.cpp must COMPILE for every part
# in PARTS with the project's own flags. There is ONE part today - the
# stratum's map (brio/ch32v00x/device.hpp) states the CH32V006K8 alone
# and no vendor header exists to select another - so the part changes
# nothing in the compile yet; the loop is where the second part and its
# tiering will land. What the sweep proves meanwhile is that WCH's gcc
# 15.2 accepts every construct brio is written with (util_all.cpp).
# Negative: every test/family_ch32v00x/neg/*.cpp must FAIL to compile
# for each part named on its "// mcu: <list>" line.
#
# No CMake coupling on purpose (same as the other three scripts): the
# compiler is called directly, the sweep takes seconds, no hardware.
#
# Usage: brio check ch32v00x            all TUs, all parts
#        brio check ch32v00x pin        only TUs/negatives matching "pin"
set -u
cd "$(dirname "$0")/../.."

CXX=/sw/wch-riscv/bin/riscv32-wch-elf-g++
FLAGS="-march=rv32ec_zmmul -mabi=ilp32e -std=gnu++23 -Os \
       -Wall -Wextra -Werror -fno-exceptions -fno-rtti -c -Ibrio"
PARTS="ch32v006k8"
FILTER="${1:-}"
fail=0

for tu in test/family_ch32v00x/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    for part in $PARTS; do
        if $CXX $FLAGS "$tu" -o /dev/null 2>/tmp/check_ch32v00x_err; then
            line="$line $part"
        else
            line="$line $part:FAIL"
            fail=1
            sed "s/^/    /" /tmp/check_ch32v00x_err | head -15
        fi
    done
    echo "POS $line"
done

for tu in test/family_ch32v00x/neg/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    parts="$(sed -n 's|^// mcu:||p' "$tu")"
    if [ -z "$parts" ]; then
        echo "NEG $(basename "$tu"): missing '// mcu:' line"; fail=1; continue
    fi
    line="$(basename "$tu" .cpp):"
    for part in $parts; do
        if $CXX $FLAGS "$tu" -o /dev/null 2>/dev/null; then
            line="$line $part:COMPILED(BAD)"
            fail=1
        else
            line="$line $part:refused"
        fi
    done
    echo "NEG $line"
done

[ "$fail" -eq 0 ] && echo "check_ch32v00x: OK" || echo "check_ch32v00x: FAILURES"
exit "$fail"
