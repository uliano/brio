"""brio check's vector guard - the ch32vx03 stratum's floating-point
promise, read off the linked images.

On the QingKe V4F (the CH32V303) the hardware prologue saves the sixteen
caller-saved integer registers and no f-register, so an image built with
the F extension binds each vector as a NAKED TRAMPOLINE - a call of an
ordinary function, the body, then MRET - and nothing saves the
f-registers of the program it interrupts (brio/ch32vx03/pfic.hpp,
BRIO_CH32_VECTOR). The promise that makes that correct - the body and
everything it reaches use no f-register - is checked here, in the image
as linked, never in the source - and so is the leaf form's, the handler
that needs no trampoline because it saves nothing:

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
f-registers and fcsr around its call (BRIO_CH32_VECTOR_FLOAT) is
checked for the set it stores and loads - every f-register back from
its own slot, fcsr saved before the call and written back after it from
a slot of its own, the stack pointer back where it was - and its body is
left alone; an entry with a `<name>_body` beside it that is neither
shape FAILS (a trampoline the guard does not recognize is one it has not
checked).

An entry bound LEAF (BRIO_CH32_LEAF_VECTOR, the attributed handler,
marked in the image by the absolute symbol `__brio_leaf_<name>`) is
checked the other way: the handler itself must be a leaf - no call, no
jump or branch out of it (a tail call), no indirect transfer but a jump
table into itself, no floating-point instruction or CSR - and FAILS
otherwise, since gcc has then saved in software what the trampoline
exists to spare; a mark with no entry of the table under its name FAILS
too. And a plain trampoline whose body, alone, is such a leaf is
REPORTED as a candidate for the leaf form - a line of the output, never
a failure.

Every other entry FAILS but the crt's own `default_handler`, the spin
every unbound vector aliases: a handler bound with an attribute of its
own, or by any shape but the stratum's three macros, is one nothing
checks - and an entry that is the start of no function fails too. Each
failure names the image, the vector, the chain of functions from its
body and the instruction. Exit status 0 when every image passes, 1
otherwise."""

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


def fcsr_read(m, ops):
    """The register a read of fcsr lands in, or None."""
    if m == "frcsr" and len(ops) == 1:
        return ops[0]
    if m == "fscsr" and len(ops) == 2:
        return ops[0]
    if m == "csrr" and ops[1:] == ["fcsr"]:
        return ops[0]
    if m in ("csrrw", "csrrs", "csrrc") and len(ops) == 3 and ops[1] == "fcsr" and ops[0] != "zero":
        return ops[0]
    return None


def fcsr_write(m, ops):
    """The register a write of fcsr takes its value from, or None."""
    if m == "fscsr" and len(ops) == 1:
        return ops[0]
    if m == "csrw" and len(ops) == 2 and ops[0] == "fcsr":
        return ops[1]
    if m == "csrrw" and len(ops) == 3 and ops[0] == "zero" and ops[1] == "fcsr":
        return ops[2]
    return None


def sp_slot(o):
    """(register, offset) of a load or store `reg,off(sp)`, or None."""
    m = re.match(r"^(\w+),(-?\d+)\(sp\)$", o.split("#")[0].strip().replace(" ", ""))
    return (m.group(1), int(m.group(2))) if m else None


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
    # The float trampoline: the twenty caller-saved f-registers stored
    # before the call and loaded after it, each from the slot it was
    # stored in; fcsr read into an integer register and stored before
    # the call, loaded and written back after it; the stack pointer
    # moved down and back up by one multiple of sixteen; nothing else.
    stored, loaded, adjust = {}, {}, 0
    saved_fcsr = restored_fcsr = None   # the stack slot each one uses
    pending = None                      # (register, slot) between the two halves of a step
    for half, seq in (("before", before), ("after", after)):
        for _a, m, o in seq:
            ops = operands_of(o)
            if m == "addi" and ops[:2] == ["sp", "sp"]:
                adjust += int(ops[2], 0)
                continue
            if m in ("fsw", "flw"):
                slot = sp_slot(o)
                if slot is None or (m == "fsw") != (half == "before"):
                    return None
                (stored if m == "fsw" else loaded)[slot[0]] = slot[1]
                continue
            if half == "before" and fcsr_read(m, ops) is not None and pending is None:
                pending = fcsr_read(m, ops)
                continue
            if half == "before" and m == "sw" and pending is not None and sp_slot(o) \
                    and sp_slot(o)[0] == pending and saved_fcsr is None:
                saved_fcsr, pending = sp_slot(o)[1], None
                continue
            if half == "after" and m == "lw" and sp_slot(o) and pending is None and restored_fcsr is None:
                pending = sp_slot(o)
                continue
            if half == "after" and isinstance(pending, tuple) and fcsr_write(m, ops) == pending[0]:
                restored_fcsr, pending = pending[1], None
                continue
            return None
        if pending is not None:
            return None
    if set(stored) != CALLER_SAVED_F or set(loaded) != CALLER_SAVED_F:
        return ("bad float", "stores %s and loads %s" % (sorted(stored), sorted(loaded)))
    if stored != loaded:
        return ("bad float", "loads an f-register from a slot it was not stored in")
    if saved_fcsr is None or restored_fcsr is None:
        return ("bad float", "stores the twenty f-registers but does not save and restore fcsr")
    if saved_fcsr != restored_fcsr or saved_fcsr in stored.values():
        return ("bad float", "restores fcsr from a slot that is not its own")
    if adjust != 0:
        return ("bad float", "leaves the stack pointer moved by %d" % adjust)
    return ("float", body)


def leaf_faults(data, funcs, fn):
    """([(addr, why, mnemonic, operands)], jump tables read) of the
    function `fn`: what makes it no leaf - a call, a jump or a branch out
    of it (a tail call), an indirect transfer that is not a jump table
    into itself, an f-register or a floating-point CSR touched. The list
    is empty for a leaf."""
    _name, end, insns = funcs[fn]
    out, tables = [], 0
    for k, (addr, m, o) in enumerate(insns):
        why = fp_fault(m, o)
        target = resolved_target(m, o)
        if why is None and m in ("jal", "call", "jalr") and not (m == "jalr" and target is None):
            why = "a call"
        if why is None and target is None:
            if m == "jr" and jump_table(data, insns, k, fn, end) is not None:
                tables += 1
                continue
            why = "an indirect %s" % ("call" if m == "jalr" else "jump")
        if why is None and isinstance(target, int) and not fn <= target < end:
            why = "a jump out of the function (a tail call)"
        if why is not None:
            out.append((addr, why, m, o))
    return out, tables


def describe(faults):
    """The first instruction of each kind of fault, with the count of the
    others of that kind."""
    parts, seen = [], {}
    for f in faults:
        seen.setdefault(f[1], []).append(f)
    for why, fs in seen.items():
        addr, _w, m, o = fs[0]
        more = " (and %d more such instructions)" % (len(fs) - 1) if len(fs) > 1 else ""
        parts.append("0x%x - %s: %s %s%s" % (addr, why, m, o, more))
    return "; ".join(parts)


LEAF_MARK = "__brio_leaf_"


def guard(elf):
    """(failures, candidates, summary line) for one image."""
    with open(elf, "rb") as f:
        data = f.read()
    name = os.path.splitext(os.path.basename(elf))[0]
    if not single_float(data):
        return [], [], "%s: not a single-float image, nothing to guard" % name
    named, sized = symbols(elf)
    funcs = listing(elf, sized)
    starts = sorted(funcs)
    marked = {n[len(LEAF_MARK):] for n in named if n.startswith(LEAF_MARK)}
    default = named.get("default_handler", (None, 0))[0]

    def owner(addr):
        i = bisect.bisect_right(starts, addr) - 1
        return starts[i] if i >= 0 and addr < funcs[starts[i]][1] else None

    failures, candidates = [], []
    walked, floats, leaves, others, reached, tables = 0, 0, 0, 0, set(), 0
    for index, entry in vector_table(elf, data, named):
        if entry == default:
            others += 1
            continue
        if entry not in funcs:
            failures.append("%s: entry %d is 0x%x, the start of no function" % (name, index, entry))
            continue
        vname, _end, insns = funcs[entry]
        kind = classify(insns)
        if vname in marked:
            # Bound LEAF: an attributed handler, which must call nothing,
            # jump out nowhere and touch no f-register - else gcc has
            # saved the twenty f-registers the trampoline exists to spare.
            marked.discard(vname)
            leaves += 1
            if kind is not None:
                failures.append("%s: vector %s (entry %d) is bound leaf and is a trampoline" % (name, vname, index))
                continue
            faults, n = leaf_faults(data, funcs, entry)
            tables += n
            if faults:
                failures.append("%s: vector %s (entry %d) is bound leaf and is none: %s"
                                % (name, vname, index, describe(faults)))
            continue
        if kind is None:
            if vname + "_body" in named:
                failures.append("%s: vector %s (entry %d) has a body, %s_body, and is not a trampoline the "
                                "guard recognizes" % (name, vname, index, vname))
            else:
                # Neither shape, no leaf mark, not the crt's spin: a
                # handler bound some other way, which nothing checks.
                failures.append("%s: vector %s (entry %d) is bound by none of the stratum's macros "
                                "(BRIO_CH32_VECTOR, BRIO_CH32_VECTOR_FLOAT, BRIO_CH32_LEAF_VECTOR)"
                                % (name, vname, index))
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
        if not leaf_faults(data, funcs, body)[0]:
            candidates.append("%s: vector %s (entry %d): its body, %s, is a leaf - BRIO_CH32_LEAF_VECTOR "
                              "binds it without the trampoline" % (name, vname, index, funcs[body][0]))
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
                failures.append("%s: vector %s (entry %d) reaches %s at %s\n    via %s"
                                % (name, vname, index, fname, describe(hits), " <- ".join(chain)))
    for vname in sorted(marked):
        failures.append("%s: %s is bound leaf and is no entry of the vector table" % (name, vname))
    summary = ("%s: %d trampolines walked over %d functions (%d jump tables read), %d leaf vectors checked, "
               "%d float trampolines checked, %d entries the crt's default handler"
               % (name, walked, len(reached), tables, leaves, floats, others))
    return failures, candidates, summary


def run(elfs, quiet=False):
    """Every image guarded: a failure printed always, a leaf candidate
    always (a report, not a failure), an image's summary unless quiet."""
    bad = ncand = 0
    for elf in elfs:
        failures, candidates, summary = guard(elf)
        if failures:
            bad += 1
            print("vector guard: FAIL " + summary)
            for f in failures:
                print("  " + f)
        elif not quiet:
            print("vector guard: " + summary)
        for c in candidates:
            print("vector guard: candidate " + c)
        ncand += len(candidates)
    print("vector guard: %d images, %d failed, %d leaf candidates" % (len(elfs), bad, ncand))
    return 1 if bad else 0


def main(argv):
    if len(argv) < 2 or argv[1] in ("-h", "--help"):
        print(__doc__)
        return 0 if len(argv) >= 2 else 2
    return run(argv[1:])


if __name__ == "__main__":
    sys.exit(main(sys.argv))
