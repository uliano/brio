"""brio check's vector guard - the ch32vx03 stratum's floating-point
promise, read off the linked images.

On the QingKe V4F (the CH32V303) the hardware prologue saves the sixteen
caller-saved integer registers and no f-register, so an image built with
the F extension binds each vector as a NAKED TRAMPOLINE - a call of an
ordinary function, the body, then MRET - and nothing saves the
f-registers of the program it interrupts (brio/ch32vx03/pfic.hpp,
BRIO_CH32_VECTOR). The promise that makes that correct - the body and
everything it reaches use no f-register - is checked here, in the image
as linked, never in the source:

    python3 -m cli.vector_guard IMAGE.elf...      (or: brio check ch32vx03)

For every image whose ELF header says the single-float ABI, the roots
are the vector table's own entries (the crt's `_start`, its length the
symbol's size, one word each after the first, which is an
instruction). An entry whose code is a trampoline - nothing but one
call and MRET - has its body walked: every
direct call, tail jump and branch out of a function is followed, and
the walk FAILS on

  - a floating-point instruction anywhere on it (a load or a store of an
    f-register, any F arithmetic, a move or a conversion, an access to
    fcsr, frm or fflags);
  - an indirect call or jump the listing does not resolve to a symbol
    (a call through a pointer can reach anything) - but for a switch's
    jump table, gcc's own shape (the index bounded by a compare against
    a constant, the table's address built with lui and addi, a word
    loaded and jumped to), whose every entry is read from the image and
    must land inside the same function;
  - an instruction the disassembler could not decode that is four bytes
    long (the two-byte ones are WCH's xw extension, which takes the slots
    of the D extension's compressed loads and stores: this core has no
    D, and every compressed F instruction it has is decoded).

An entry that is a trampoline storing the twenty caller-saved
f-registers around its call (BRIO_CH32_VECTOR_FLOAT) is checked for the
set it stores and loads and its body is left alone; an entry with a
`<name>_body` beside it that is neither shape FAILS (a trampoline the
guard does not recognize is one it has not checked); any other entry -
the crt's default handler, a handler bound with an attribute of its own
- is not a trampoline and owes the guard nothing. Each failure names the
image, the vector, the chain of functions from its body and the
instruction. Exit status 0 when every image passes, 1 otherwise."""

import bisect
import os
import re
import struct
import subprocess
import sys

OBJDUMP = "/sw/wch-riscv/bin/riscv32-wch-elf-objdump"
EF_RISCV_FLOAT_ABI = 0x6
EF_RISCV_FLOAT_ABI_SINGLE = 0x2
# The twenty f-registers the ilp32f ABI lets a callee clobber.
CALLER_SAVED_F = {"ft%d" % i for i in range(12)} | {"fa%d" % i for i in range(8)}
BRANCHES = {"beq", "bne", "blt", "bge", "bltu", "bgeu", "beqz", "bnez", "blez", "bgez", "bltz",
            "bgtz", "bgt", "ble", "bgtu", "bleu"}
FP_CSRS = ("fflags", "frm", "fcsr")
# Major opcodes of the F extension's 32-bit instructions: LOAD-FP,
# STORE-FP, the four fused multiply-adds, OP-FP.
FP_OPCODES = {0x07, 0x27, 0x43, 0x47, 0x4B, 0x4F, 0x53}

LINE = re.compile(r"^\s*([0-9a-f]+):\s+(\S+)(?:\s+(.*))?$")
HEAD = re.compile(r"^([0-9a-f]+) <(.+)>:$")
TARGET = re.compile(r"(?:^|[\s,])([0-9a-f]+) <([^>]+)>")


def single_float(data):
    (e_flags,) = struct.unpack_from("<I", data, 0x24)
    return (e_flags & EF_RISCV_FLOAT_ABI) == EF_RISCV_FLOAT_ABI_SINGLE


def read_words(data, addr, count):
    """`count` little-endian words of the image at a load address, or
    None when no loaded segment holds them."""
    (e_phoff,) = struct.unpack_from("<I", data, 0x1C)
    (e_phentsize, e_phnum) = struct.unpack_from("<HH", data, 0x2A)
    for i in range(e_phnum):
        (_t, off, vaddr, _p, filesz, _m) = struct.unpack_from("<IIIIII", data, e_phoff + i * e_phentsize)
        if vaddr <= addr and addr + 4 * count <= vaddr + filesz:
            raw = data[off + addr - vaddr:off + addr - vaddr + 4 * count]
            return [w for (w,) in struct.iter_unpack("<I", raw)]
    return None


def symbols(elf):
    """{name: (address, size)} of every symbol, and {address: size} of
    the functions (the largest size given at an address)."""
    text = subprocess.run([OBJDUMP, "-t", elf], capture_output=True, text=True, check=True).stdout
    named, sized = {}, {}
    for line in text.splitlines():
        m = re.match(r"^([0-9a-f]{8}) (.{7}) (\S+)\s+([0-9a-f]{8}) (.+)$", line)
        if not m:
            continue
        addr, flags, size, name = int(m.group(1), 16), m.group(2), int(m.group(4), 16), m.group(5).strip()
        named[name] = (addr, size)
        if "F" in flags:
            sized[addr] = max(size, sized.get(addr, 0))
    return named, sized


def vector_table(elf, data, named):
    """[(index, address)] of the table's nonzero entries after the first."""
    start, size = named.get("_start", (0, 0))
    if size == 0:
        raise SystemExit("vector guard: %s has no sized _start (the crt's vector table)" % elf)
    words = read_words(data, start, size // 4)
    if words is None:
        raise SystemExit("vector guard: %s: the vector table lies in no loaded segment" % elf)
    return [(i, w) for i, w in enumerate(words) if i and w]


def listing(elf, sized):
    """{start: (name, end, [(addr, mnemonic, operands)])} of every
    function the disassembler names, cut at the symbol's size where it
    has one (the read-only data the linker places after a function is
    not its code)."""
    text = subprocess.run([OBJDUMP, "-d", "-C", "--no-show-raw-insn", elf], capture_output=True, text=True,
                          check=True).stdout
    funcs, cur = {}, None
    for line in text.splitlines():
        h = HEAD.match(line)
        if h:
            cur = int(h.group(1), 16)
            funcs[cur] = [h.group(2), None, []]
            continue
        m = LINE.match(line)
        if m and cur is not None:
            funcs[cur][2].append((int(m.group(1), 16), m.group(2), (m.group(3) or "").strip()))
    out = {}
    for addr, (name, _e, insns) in funcs.items():
        size = sized.get(addr, 0)
        if size:
            insns = [i for i in insns if i[0] < addr + size]
            end = addr + size
        else:
            end = insns[-1][0] + 4 if insns else addr
        out[addr] = (name, end, insns)
    return out


def operands_of(o):
    return [p.strip() for p in o.split("#")[0].split(",")]


def last_writer(window, reg, before):
    """The last instruction of window[:before] that writes `reg`, as
    (index, mnemonic, operands); None when there is none or when an
    instruction the disassembler did not decode (which may write any
    register) comes first."""
    for i in range(before - 1, -1, -1):
        _a, m, o = window[i]
        if m == ".insn":
            return None
        ops = operands_of(o)
        stores = m in ("sb", "sh", "sw", "fsw") or m in BRANCHES
        if ops[0] == reg and not stores:
            return i, m, o
    return None


# Instructions whose result is no larger, unsigned, than their source:
# a bound on the source is a bound on the result.
NARROWING = ("zext.b", "zext.h", "mv", "andi")


def jump_table(data, insns, k, start, end):
    """The targets of the jump table the `jr` at insns[k] dispatches
    through, when it has gcc's shape - a word loaded from base + 4 x
    index, the base a constant, the index bounded by an unsigned compare
    against a constant, no branch landing between the compare and the
    jump - and every entry lands in [start, end); None otherwise."""
    window = insns[max(0, k - 16):k]
    reg = operands_of(insns[k][2])[0]
    lw = last_writer(window, reg, len(window))
    if lw is None or lw[1] != "lw":
        return None
    m = re.match(r"^0\((\w+)\)$", operands_of(lw[2])[1])
    if not m:
        return None
    add = last_writer(window, m.group(1), lw[0])
    if add is None or add[1] != "add":
        return None
    base = index = None
    first = add[0]
    for src in operands_of(add[2])[1:3]:
        w = last_writer(window, src, add[0])
        if w is None:
            return None
        wi, wm, wo = w
        first = min(first, wi)
        t = TARGET.search(wo)
        if wm == "li":
            base = int(operands_of(wo)[1], 0)
        elif wm == "addi" and t:
            base = int(t.group(1), 16)
        elif wm == "slli" and operands_of(wo)[2] in ("0x2", "2"):
            index = (wi, operands_of(wo)[1])
    if base is None or index is None:
        return None
    # The index, and every register it was narrowed from.
    names, at = {index[1]}, index[0]
    while True:
        w = last_writer(window, index[1], at)
        if w is None or w[1] not in NARROWING:
            break
        at = w[0]
        names.add(operands_of(w[2])[1])
        index = (at, operands_of(w[2])[1])
    count = None
    for i, (_a, bm, bo) in enumerate(window):
        ops = operands_of(bo)
        # bltu K, i jumps away when K < i, so i <= K falls through: K + 1
        # entries; bgeu i, K jumps away when i >= K: K entries.
        bound = (ops[0], 1) if bm == "bltu" and len(ops) > 1 and ops[1] in names else \
            (ops[1], 0) if bm == "bgeu" and ops[0] in names else None
        if bound is not None:
            w = last_writer(window, bound[0], i)
            if w is not None and w[1] == "li":
                count = int(operands_of(w[2])[1], 0) + bound[1]
                first = min(first, w[0])
    if count is None:
        return None
    lo, hi = window[first][0], insns[k][0]
    for _a, bm, bo in insns:
        target = resolved_target(bm, bo)
        if isinstance(target, int) and lo < target <= hi:
            return None
    words = read_words(data, base, count)
    if words is None or any(not start <= w < end for w in words):
        return None
    return words


def resolved_target(mnemonic, operands):
    """The address a control transfer names, or None for an indirect one;
    'no' for an instruction that transfers nothing out of the line."""
    if mnemonic in ("ret", "mret"):
        return "no"
    if mnemonic in ("j", "jal", "jalr", "jr", "tail", "call") or mnemonic in BRANCHES:
        t = TARGET.search(operands)
        return int(t.group(1), 16) if t else None
    return "no"


def fp_fault(mnemonic, operands):
    """Why this instruction touches the floating-point unit, or None."""
    if mnemonic.startswith("f") and not mnemonic.startswith("fence"):
        return "a floating-point instruction"
    if mnemonic.startswith("csr") and any(re.search(r"\b%s\b" % c, operands) for c in FP_CSRS):
        return "an access to the floating-point CSRs"
    if mnemonic == ".insn":
        size, word = [p.strip() for p in operands.split("#")[0].split(",")[:2]]
        if size == "4":
            if int(word, 0) & 0x7F in FP_OPCODES:
                return "an undecoded instruction with a floating-point opcode"
            return "a four-byte instruction the disassembler does not decode"
    return None


def classify(insns):
    """('plain', body) / ('float', body) / ('bad float', why) for a
    trampoline, None for anything else."""
    calls = [(i, resolved_target(m, o)) for i, (_a, m, o) in enumerate(insns)
             if m in ("jal", "call") or (m == "jalr" and TARGET.search(o))]
    if not insns or insns[-1][1] != "mret" or len(calls) != 1 or calls[0][1] in (None, "no"):
        return None
    k, body = calls[0]
    before, after = insns[:k], insns[k + 1:-1]
    # A call the linker could not relax is auipc ra and jalr ra.
    if before and before[-1][1] == "auipc" and before[-1][2].startswith("ra,"):
        before = before[:-1]
    if not before and not after:
        return ("plain", body)
    shape = re.compile(r"^(f[sl]w)$")
    stored = {o.split(",")[0] for _a, m, o in before if m == "fsw"}
    loaded = {o.split(",")[0] for _a, m, o in after if m == "flw"}
    others = [(m, o) for _a, m, o in before + after
              if not shape.match(m) and not (m == "addi" and o.startswith("sp,sp,"))]
    if others:
        return None
    if stored != CALLER_SAVED_F or loaded != CALLER_SAVED_F:
        return ("bad float", "stores %s and loads %s" % (sorted(stored), sorted(loaded)))
    return ("float", body)


def guard(elf):
    """(failures, summary line) for one image."""
    with open(elf, "rb") as f:
        data = f.read()
    name = os.path.splitext(os.path.basename(elf))[0]
    if not single_float(data):
        return [], "%s: not a single-float image, nothing to guard" % name
    named, sized = symbols(elf)
    funcs = listing(elf, sized)
    starts = sorted(funcs)

    def owner(addr):
        i = bisect.bisect_right(starts, addr) - 1
        return starts[i] if i >= 0 and addr < funcs[starts[i]][1] else None

    failures, walked, floats, others, reached, tables = [], 0, 0, 0, set(), 0
    for index, entry in vector_table(elf, data, named):
        if entry not in funcs:
            others += 1
            continue
        vname, _end, insns = funcs[entry]
        kind = classify(insns)
        if kind is None:
            if vname + "_body" in named:
                failures.append("%s: vector %s (entry %d) has a body, %s_body, and is not a trampoline the "
                                "guard recognizes" % (name, vname, index, vname))
            others += 1
            continue
        if kind[0] == "bad float":
            failures.append("%s: vector %s (entry %d) is a float trampoline that %s" % (name, vname, index, kind[1]))
            continue
        if kind[0] == "float":
            floats += 1
            continue
        walked += 1
        body = owner(kind[1])
        if body is None:
            failures.append("%s: vector %s (entry %d) calls 0x%x, inside no function" % (name, vname, index, kind[1]))
            continue
        parent = {body: None}
        todo = [body]
        while todo:
            fn = todo.pop()
            reached.add(fn)
            fname, fend, finsns = funcs[fn]
            hits = []
            for k, (addr, m, o) in enumerate(finsns):
                why = fp_fault(m, o)
                target = resolved_target(m, o)
                if why is None and target is None:
                    if m == "jr" and jump_table(data, finsns, k, fn, fend) is not None:
                        tables += 1
                        continue
                    why = "an indirect %s the listing does not resolve" % ("call" if m == "jalr" else "jump")
                if why is not None:
                    hits.append((addr, why, m, o))
                    continue
                if target == "no":
                    continue
                dest = owner(target)
                if dest is None:
                    failures.append("%s: vector %s (entry %d): %s at 0x%x jumps to 0x%x, inside no function"
                                    % (name, vname, index, fname, addr, target))
                elif dest != fn and dest not in parent:
                    parent[dest] = fn
                    todo.append(dest)
            if hits:
                chain, f = [], fn
                while f is not None:
                    chain.append(funcs[f][0])
                    f = parent[f]
                addr, why, m, o = hits[0]
                more = " (and %d more such instructions)" % (len(hits) - 1) if len(hits) > 1 else ""
                failures.append("%s: vector %s (entry %d) reaches %s at 0x%x - %s: %s %s%s\n    via %s"
                                % (name, vname, index, fname, addr, why, m, o, more, " <- ".join(chain)))
    summary = ("%s: %d trampolines walked over %d functions (%d jump tables read), %d float trampolines "
               "left alone, %d entries not trampolines" % (name, walked, len(reached), tables, floats, others))
    return failures, summary


def run(elfs, quiet=False):
    bad = 0
    for elf in elfs:
        failures, summary = guard(elf)
        if failures:
            bad += 1
            print("vector guard: FAIL " + summary)
            for f in failures:
                print("  " + f)
        elif not quiet:
            print("vector guard: " + summary)
    print("vector guard: %d images, %d failed" % (len(elfs), bad))
    return 1 if bad else 0


def main(argv):
    if len(argv) < 2 or argv[1] in ("-h", "--help"):
        print(__doc__)
        return 0 if len(argv) >= 2 else 2
    return run(argv[1:])


if __name__ == "__main__":
    sys.exit(main(sys.argv))
