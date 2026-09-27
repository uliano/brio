#!/usr/bin/env bash
# Family compile check (part of every driver's definition of done).
#
# Positive: every test/family/*.cpp must COMPILE for every package of
# the AVR DA/DB family (the bench chip alone masks half the family:
# missing ports, instances, registers, enum values).
# Negative: every test/family/neg/*.cpp must FAIL to compile for each
# MCU named on its "// mcu: <list>" line (what must be refused must be
# refused at compile time).
# Link: a program calling every name avrdx/cmake/avr-refused-libc.rsp
# refuses must FAIL to link, each name refused, for every MCU.
#
# Usage: brio check avrdx            all TUs, all MCUs
#        brio check avrdx tcb        only TUs/negatives matching "tcb"
set -u
cd "$(dirname "$0")/../.."

CXX=/sw/avr/bin/avr-g++
# A compiler that is not installed must not pass for a refusal: a
# command that cannot start fails like a TU that failed to compile, so
# every negative would read "refused" and every positive would fail
# for a reason that is not the code's. Say so and stop.
[ -x "$CXX" ] || { echo "$(basename "$0"): the compiler $CXX is not installed - nothing checked" >&2; exit 2; }
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

# The refused C library (design/runtime.md): a program that references
# every name avrdx/cmake/avr-refused-libc.rsp wraps, and allocates with
# new, must FAIL to link with the same response file - each wrapped name
# reported as an undefined __wrap_<name>, and operator new undefined
# because this toolchain's link gives it no definition.
RSP=avrdx/cmake/avr-refused-libc.rsp
case "libc" in *"$FILTER"*)
    names="$(sed -n 's/^--wrap=//p' "$RSP" 2>/dev/null)"
    # An empty list would refuse nothing and every link below would still
    # fail on operator new: say so instead of passing.
    if [ -z "$names" ]; then
        echo "LINK refused-libc: $RSP names nothing - nothing checked"; fail=1
    fi
    tu=/tmp/check_family_libc.cpp
    {
        for n in $names; do echo "extern \"C\" void $n();"; done
        echo "char* volatile brio_kept;"
        echo "int main() {"
        for n in $names; do echo "    $n();"; done
        echo "    brio_kept = new char;"
        echo "    return 0;"
        echo "}"
    } > "$tu"
    line="refused-libc ($(echo $names | wc -w | tr -d ' ') names + operator new):"
    for mcu in $MCUS; do
        if $CXX -mmcu="$mcu" -std=gnu++23 -Os -fno-builtin -w "$tu" -o /tmp/check_family_libc.elf \
                -Wl,@"$RSP" 2>/tmp/check_family_err; then
            line="$line $mcu:LINKED(BAD)"; fail=1; continue
        fi
        missing=""
        for n in $names; do
            grep -q "undefined reference to \`__wrap_$n'" /tmp/check_family_err || missing="$missing $n"
        done
        grep -q "undefined reference to \`operator new" /tmp/check_family_err || missing="$missing new"
        if [ -n "$missing" ]; then
            line="$line $mcu:NOT-REFUSED($missing )"; fail=1
        else
            line="$line $mcu:refused"
        fi
    done
    rm -f "$tu" /tmp/check_family_libc.elf
    echo "LINK $line"
;; esac

[ "$fail" -eq 0 ] && echo "check_family: OK" || echo "check_family: FAILURES"
exit "$fail"
