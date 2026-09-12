#!/usr/bin/env bash
# Family compile check for the RP2040 stratum (part of every driver's
# definition of done, the rp2040 twin of cli/checks/check_stm32g0.sh).
#
# Positive: every test/family_rp2040/*.cpp must COMPILE with the
# project's own flags against the one device header this family has -
# the RP2040 is one chip in one package, so "the family" is the chip
# and the sweep proves what a bench image cannot: that every header of
# the stratum instantiates every verb it offers, including the ones no
# app has called yet.
# Negative: every test/family_rp2040/neg/*.cpp must FAIL to compile
# (what must be refused must be refused at compile time).
#
# No CMake coupling on purpose (same as the other scripts): the compiler
# is called directly, the sweep takes seconds and needs no hardware.
#
# Usage: brio check rp2040            all TUs
#        brio check rp2040 uart       only TUs/negatives matching "uart"
set -u
cd "$(dirname "$0")/../.."

CXX=/sw/arm-none-eabi/bin/arm-none-eabi-g++
FLAGS="-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -std=gnu++23 -Os \
       -Wall -Wextra -Werror -fno-exceptions -fno-rtti -c \
       -Ibrio -Ithird_party/pico-sdk/CMSIS -Ithird_party/pico-sdk -Ithird_party/cmsis-core"
FILTER="${1:-}"
fail=0

for tu in test/family_rp2040/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    if $CXX $FLAGS "$tu" -o /dev/null 2>/tmp/check_rp2040_err; then
        line="$line rp2040"
    else
        line="$line rp2040:FAIL"
        fail=1
        sed "s/^/    /" /tmp/check_rp2040_err | head -15
    fi
    echo "POS $line"
done

for tu in test/family_rp2040/neg/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    if $CXX $FLAGS "$tu" -o /dev/null 2>/dev/null; then
        line="$line rp2040:COMPILED(BAD)"
        fail=1
    else
        line="$line rp2040:refused"
    fi
    echo "NEG $line"
done

[ "$fail" -eq 0 ] && echo "check_rp2040: OK" || echo "check_rp2040: FAILURES"
exit "$fail"
