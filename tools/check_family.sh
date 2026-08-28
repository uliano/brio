#!/usr/bin/env bash
# Family compile check (part of every driver's definition of done).
#
# Positive: every test/family/*.cpp must COMPILE for every package of
# the AVR DA/DB family (the bench chip alone masks half the family:
# missing ports, instances, registers, enum values).
# Negative: every test/family/neg/*.cpp must FAIL to compile for each
# MCU named on its "// mcu: <list>" line (what must be refused must be
# refused at compile time).
#
# Usage: tools/check_family.sh            all TUs, all MCUs
#        tools/check_family.sh tcb        only TUs/negatives matching "tcb"
set -u
cd "$(dirname "$0")/.."

CXX=/sw/avr/bin/avr-g++
FLAGS="-std=gnu++23 -Os -Wall -Wextra -Werror -c -Ibrio"
MCUS="avr128db28 avr128db32 avr128db48 avr128db64 \
      avr128da28 avr128da32 avr128da48 avr128da64"
FILTER="${1:-}"
fail=0

for tu in test/family/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    for mcu in $MCUS; do
        if $CXX -mmcu="$mcu" $FLAGS "$tu" -o /dev/null 2>/tmp/check_family_err; then
            line="$line $mcu"
        else
            line="$line $mcu:FAIL"
            fail=1
            sed "s/^/    /" /tmp/check_family_err | head -15
        fi
    done
    echo "POS $line"
done

for tu in test/family/neg/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    mcus="$(sed -n 's|^// mcu:||p' "$tu")"
    if [ -z "$mcus" ]; then
        echo "NEG $(basename "$tu"): missing '// mcu:' line"; fail=1; continue
    fi
    line="$(basename "$tu" .cpp):"
    for mcu in $mcus; do
        if $CXX -mmcu="$mcu" $FLAGS "$tu" -o /dev/null 2>/dev/null; then
            line="$line $mcu:COMPILED(BAD)"
            fail=1
        else
            line="$line $mcu:refused"
        fi
    done
    echo "NEG $line"
done

[ "$fail" -eq 0 ] && echo "check_family: OK" || echo "check_family: FAILURES"
exit "$fail"
