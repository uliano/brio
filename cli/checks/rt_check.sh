# The runtime's part of the family checks (design/runtime.md), the same for
# every 32-bit family and sourced by each family's script, which passes its
# own compiler and flags - so the runtime is checked by every compiler that
# builds it, for every ISA and ABI a family builds for. Not a check of its
# own: `brio check <stratum>` runs it, or `brio check <stratum> rt` alone.
#
# rt_check <label> <cxx> <flags...>
#   brio/rt/rt.cpp must compile warning-free with the family's flags, at the
#   -Os the images use AND at -O3, where GCC's loop recognition turns the
#   file's copy and fill loops into calls to memcpy and memset when the
#   file's guard is missing (measured) - and in both objects no call
#   relocation may name one of the seven functions the file defines: that
#   would be a function calling itself.
#
# ram_text_check <label> <cxx> <tu> <flags...>
#   A family whose flash driver runs code from RAM while the flash is
#   disconnected (the section .ram_text) compiles that driver's fixture and
#   no relocation of .ram_text may name one of the seven: they live in
#   flash, and a call to one from there would fault.
#
# Both print one line and return non-zero on a failure.

RT_SEVEN='memcpy|memmove|memset|memcmp|strlen|memchr|abort'

# The readelf beside a compiler: <prefix>-g++ -> <prefix>-readelf.
rt_readelf() { printf '%s\n' "${1%g++}readelf"; }

rt_check() {
    local label="$1" cxx="$2"; shift 2
    local re; re="$(rt_readelf "$cxx")"
    local obj=/tmp/brio_rt_check.o line="RT $label:" ok=0 opt calls
    for opt in -Os -O3; do
        if ! "$cxx" "$@" "$opt" -c brio/rt/rt.cpp -o "$obj" 2>/tmp/brio_rt_check_err; then
            line="$line $opt:COMPILE-FAIL"; ok=1
            sed "s/^/    /" /tmp/brio_rt_check_err | head -15
            continue
        fi
        calls="$("$re" -rW "$obj" \
            | grep -E 'R_(ARM_THM_CALL|ARM_THM_JUMP24|ARM_CALL|ARM_JUMP24|RISCV_CALL|RISCV_CALL_PLT|RISCV_JAL)' \
            | grep -o -w -E "$RT_SEVEN" | sort -u | tr '\n' ' ')"
        if [ -n "$calls" ]; then
            line="$line $opt:CALLS-ITSELF($calls)"; ok=1
        else
            line="$line $opt"
        fi
    done
    rm -f "$obj"
    echo "$line"
    return "$ok"
}

ram_text_check() {
    local label="$1" cxx="$2" tu="$3"; shift 3
    local re; re="$(rt_readelf "$cxx")"
    local obj=/tmp/brio_ram_text_check.o calls
    if ! "$cxx" "$@" -c "$tu" -o "$obj" 2>/tmp/brio_rt_check_err; then
        echo "RAM $label: $(basename "$tu"):COMPILE-FAIL"
        sed "s/^/    /" /tmp/brio_rt_check_err | head -15
        return 1
    fi
    if ! "$re" -SW "$obj" | grep -q '\.ram_text'; then
        echo "RAM $label: $(basename "$tu"):NO-.ram_text (the fixture no longer instantiates the RAM code)"
        rm -f "$obj"; return 1
    fi
    # Only the relocation sections that belong to .ram_text: readelf prints
    # a header per section, so keep the lines between such a header and the
    # next one.
    calls="$("$re" -rW "$obj" \
        | awk "/^Relocation section '\\.rela?\\.ram_text/ {on = 1; next} /^Relocation section/ {on = 0} on" \
        | grep -o -w -E "$RT_SEVEN" | sort -u | tr '\n' ' ')"
    rm -f "$obj"
    if [ -n "$calls" ]; then
        echo "RAM $label: .ram_text CALLS $calls- code that runs with the flash disconnected"
        return 1
    fi
    echo "RAM $label: .ram_text calls none of the seven"
}
