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
# own prologue), since pfic.hpp's BRIO_CH32_VECTOR is one spelling
# with several expansions (the attributed handler of either prologue,
# and on an F part with the hardware prologue the naked trampoline) and
# a fixture that binds a vector proves each.
# Negative: every test/family_ch32vx03/neg/*.cpp must FAIL to compile
# for each part named on its "// mcu: <list>" line.
# The vector guard's own proof: every test/family_ch32vx03/guard/*.cpp
# is LINKED into an image for the CH32V303VC with the hardware prologue
# (the one build whose vectors are trampolines or leaves) and
# cli/vector_guard.py run over it, which must pass or fail as the TU's
# "// guard:" line says - "pass", "pass <text>" with <text> in the
# guard's report (a leaf candidate it must name), or "fail <text>" with
# <text> in the report (the function or the vector it must name). The
# guard over the project's own images runs after the link guard
# (cli/check.py).
#
# No CMake coupling on purpose (same as the other six scripts): the
# compiler is called directly, the sweep takes seconds, no hardware.
#
# Usage: brio check ch32vx03            all TUs, all parts
#        brio check ch32vx03 pin        only TUs/negatives matching "pin"
set -u
cd "$(dirname "$0")/../.."

CXX=/sw/wch-riscv/bin/riscv32-wch-elf-g++
# A compiler that is not installed must not pass for a refusal: a
# command that cannot start fails like a TU that failed to compile, so
# every negative would read "refused" and every positive would fail
# for a reason that is not the code's. Say so and stop.
[ -x "$CXX" ] || { echo "$(basename "$0"): the compiler $CXX is not installed - nothing checked" >&2; exit 2; }
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

# The vector guard (cli/vector_guard.py) on images built to pass it and
# to fail it: a float trampoline's body, a jump table and a call chain
# it must let through, and a leaf vector with a jump table of its own; a
# trampoline whose body is a leaf it must report; a float operation
# reached through a call, a call through a pointer, a leaf vector that
# calls out or does float work, a float trampoline that leaves fcsr
# unsaved and a handler attributed by hand it must refuse, by name. Runs unfiltered or as
# `brio check ch32vx03 guard`.
case "guard" in *"$FILTER"*)
    GUARD_ELF=/tmp/check_ch32vx03_guard.elf
    LINK="-DCH32V303VC -DBRIO_CH32_HPE=1 $V4F ${COMMON/ -c / } -ffunction-sections -fdata-sections
          -nostartfiles -nodefaultlibs -Wl,--gc-sections -T ch32vx03/ld/ch32v303vc.ld"
    for tu in test/family_ch32vx03/guard/*.cpp; do
        [ -e "$tu" ] || continue
        want="$(sed -n 's|^// guard: ||p' "$tu")"
        line="$(basename "$tu" .cpp):"
        if ! $CXX $LINK ch32vx03/src/glue/startup_ch32vx03.S "$tu" -lgcc -o "$GUARD_ELF" 2>/tmp/check_ch32vx03_err; then
            echo "GUARD $line does not link"; sed "s/^/    /" /tmp/check_ch32vx03_err | head -15
            fail=1; continue
        fi
        report="$(python3 -m cli.vector_guard "$GUARD_ELF")"; status=$?
        bad=0
        case "$want" in
            pass) [ "$status" -eq 0 ] && line="$line passed" || { line="$line REFUSED(BAD)"; bad=1; } ;;
            pass\ *)
                text="${want#pass }"
                if [ "$status" -eq 0 ] && printf '%s' "$report" | grep -qF "$text"; then
                    line="$line passed, reporting '$text'"
                else
                    line="$line NOT PASSED AND REPORTED AS DUE(BAD)"; bad=1
                fi ;;
            fail\ *)
                text="${want#fail }"
                if [ "$status" -ne 0 ] && printf '%s' "$report" | grep -qF "$text"; then
                    line="$line refused, naming '$text'"
                else
                    line="$line NOT REFUSED AS DUE(BAD)"; bad=1
                fi ;;
            *) line="$line missing '// guard: pass [<text>]|fail <text>' line"; bad=1 ;;
        esac
        echo "GUARD $line"
        [ "$bad" -eq 0 ] || { printf '%s\n' "$report" | sed "s/^/    /"; fail=1; }
    done
    ;;
esac

# The runtime (design/runtime.md), over this family's own compiler and
# flags: cli/checks/rt_check.sh. Runs unfiltered or as `brio check ch32vx03 rt`.
case "rt" in *"$FILTER"*)
    . cli/checks/rt_check.sh
    rt_check v4b "$CXX" $V4B $COMMON || fail=1
    rt_check v4f "$CXX" $V4F $COMMON || fail=1
    ;;
esac

[ "$fail" -eq 0 ] && echo "check_ch32vx03: OK" || echo "check_ch32vx03: FAILURES"
exit "$fail"
