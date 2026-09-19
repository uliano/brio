#!/usr/bin/env bash
# Family compile check for the RP2350 stratum - and the first fixture of
# this project that crosses TWO COMPILERS, because this chip has two
# processor architectures over one set of peripherals and every header of
# the stratum must compile for both (this stratum's rule: one source, two
# ISAs, and a suite is green when it is green on both).
#
# THE SWEEP is therefore four builds of every TU: the Cortex-M33 and the
# Hazard3 compilers, each for both packages - the QFN-80 with 48 GPIO and
# eight ADC inputs, the QFN-60 with 30 and four (the package is a -D, and
# what it decides is stated once in brio/rp2350/device.hpp). The RISC-V
# half puts third_party/pico-sdk/rp2350/no_core FIRST on the include
# path, where the device header's unconditional include of core_cm33.h
# resolves to a stub; no other build ever sees that directory.
#
# Positive: every test/family_rp2350/*.cpp must COMPILE in all four.
# Negative: every test/family_rp2350/neg/*.cpp must FAIL to compile -
# in the ARM and the RISC-V build of the package the refusal is about.
# A negative whose name begins with `qfn60_` is a refusal the SMALLER
# package makes (a pad the QFN-60 has not got, which the QFN-80 brings
# out perfectly well), so it is built for that package; every other
# negative is built for the QFN-80.
#
# No CMake coupling on purpose (same as the other scripts): the compilers
# are called directly, the sweep takes seconds and needs no hardware.
#
# Usage: brio check rp2350            all TUs
#        brio check rp2350 pin        only TUs/negatives matching "pin"
set -u
cd "$(dirname "$0")/../.."

ARM_CXX=/sw/arm-none-eabi/bin/arm-none-eabi-g++
RV_CXX=/sw/riscv32-unknown-elf/bin/riscv32-unknown-elf-g++

COMMON="-std=gnu++23 -Os -Wall -Wextra -Werror -fno-exceptions -fno-rtti -c \
        -DBRIO_RP2350_FLASH_KB=16384 \
        -Ibrio -Ithird_party/pico-sdk/rp2350/CMSIS -Ithird_party/pico-sdk/rp2350"
ARM_FLAGS="-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 $COMMON \
           -Ithird_party/cmsis-core"
RV_FLAGS="-march=rv32ima_zicsr_zifencei_zba_zbb_zbs_zbkb_zca_zcb_zcmp -mabi=ilp32 $COMMON \
          -Ithird_party/pico-sdk/rp2350/no_core"

FILTER="${1:-}"
fail=0

# compile <tu> <arch> <pins> -> 0 when it compiled
compile() {
    case "$2" in
        arm) $ARM_CXX $ARM_FLAGS -DBRIO_RP2350_PACKAGE_PINS="$3" "$1" -o /dev/null 2>/tmp/check_rp2350_err ;;
        *)   $RV_CXX  $RV_FLAGS  -DBRIO_RP2350_PACKAGE_PINS="$3" "$1" -o /dev/null 2>/tmp/check_rp2350_err ;;
    esac
}

for tu in test/family_rp2350/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    for arch in arm riscv; do
        for pins in 80 60; do
            if compile "$tu" "$arch" "$pins"; then
                line="$line $arch/$pins"
            else
                line="$line $arch/$pins:FAIL"
                fail=1
                sed "s/^/    /" /tmp/check_rp2350_err | head -15
            fi
        done
    done
    echo "POS $line"
done

for tu in test/family_rp2350/neg/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    base="$(basename "$tu" .cpp)"
    case "$base" in qfn60_*) pins=60 ;; *) pins=80 ;; esac
    line="$base:"
    for arch in arm riscv; do
        if compile "$tu" "$arch" "$pins"; then
            line="$line $arch/$pins:COMPILED(BAD)"
            fail=1
        else
            line="$line $arch/$pins:refused"
        fi
    done
    echo "NEG $line"
done

[ "$fail" -eq 0 ] && echo "check_rp2350: OK" || echo "check_rp2350: FAILURES"
exit "$fail"
