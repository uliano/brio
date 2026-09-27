# The runtime: what an image takes from the toolchain

`rt/rt.cpp` - `memcpy`, `memmove`, `memset`, `memcmp`, `strlen`, `memchr`,
`abort`.

## What it is for

A brio image on a 32-bit target takes from its toolchain exactly one
library, libgcc - the compiler's own helpers: 64-bit division, count
leading zeros, the Arm case tables. Nothing of the C library enters.
Seven functions that a freestanding C++ program cannot avoid are brio's
own, in `rt/rt.cpp`, the one source file of the framework.

They cannot be avoided because the COMPILER and the LIBRARY HEADERS call
them, not the code:

- GCC turns a copy or a value-initialization of an object above a small
  size into a call to `memcpy` or `memset` (the threshold is low at -Os,
  where a call is smaller than the loads and stores), a library
  algorithm on trivially copyable data into `memmove` or `memcmp`, and a
  loop it recognizes as a copy or a fill into the same calls. GCC's
  manual requires these four of any freestanding environment for that
  reason. In brio's images they copy the event a dispatch receives, the
  bus requests, the packets a USB interrupt moves.
- the freestanding libstdc++ brio builds on (overview.md, "Design
  pillars") calls two more through `std::char_traits<char>`: `length`
  becomes `strlen`, so a `std::string_view` made from a pointer whose
  contents the compiler cannot see needs it, and `find` becomes
  `memchr`. The other traits are the four above.
- libstdc++ compiled with -fno-exceptions turns every throw into a call
  to `abort()`. At -Os the optimizer proves those branches dead; at -Og
  it does not, so a Debug image references `abort()` where a Release one
  does not.

Measured over every image of every 32-bit build project, Release and
Debug, linked with no C library: `memcpy`, `memset` and `memcmp` are
referenced, `strlen` by an image that makes a `std::string_view` from a
run-time pointer, `abort` in Debug. `memmove` and `memchr` are not
referenced today and are provided because the compiler and the library
may emit them. Nothing else of the C library is.

## Why brio's own, and not the C library's

The toolchains ship newlib in two variants, and the link step of each
toolchain follows its own tradition:

- newlib-nano's `mem*` are byte loops: 14..36 bytes each, and a 48-byte
  event copied one byte at a time; full newlib's copy words, and are
  ten to twenty-five times larger (up to 378 bytes for one function).
  Neither is right everywhere: a 16 KB part wants the small ones, and a
  copy on the path every event takes wants words.
- Under the C library sits the syscall layer (libgloss): `_write`,
  `_sbrk`, `_exit` and the rest. arm-none-eabi links none of it unless a
  specs file asks, so a `printf` that sneaks in fails the link there;
  both RISC-V toolchains link it always, nano or full, as syscalls made
  with `ecall` - the mistake links, and surfaces as a trap when the code
  runs. A Debug image's `abort()` alone is enough to pull that chain in
  (abort -> raise -> the signal machinery -> stdio -> malloc -> eight
  syscalls) wherever nothing else defines `abort`.

Taking the seven functions from brio instead makes all of that one
rule: the same link line on every toolchain, the size and the speed
decided once in one file, and a C-library call that cannot link
anywhere. The functions are small enough that owning them costs less
than choosing, per target, among variants that are each wrong
somewhere.

## The link line

Every 32-bit build project links its images with:

- `-nostartfiles` - the crt is the project's own (`src/glue/`), which
  also keeps the syscall layer's `crt0.o` out;
- `-nodefaultlibs` - the driver adds no default library at all: not the
  C library, not the syscall layer (they come from the same specs rule),
  not libgcc;
- `rt/rt.cpp` compiled into every image, beside the crt;
- `-lgcc`, named explicitly and placed after the objects.

No `--specs` file is used: what a specs file would select no longer
reaches the link. The toolchain's headers are untouched (`<cstring>`,
`<cstdint>` and the rest resolve as before: the options above are link
options). An accidental `printf` or `malloc` now fails the link with the
function's own name, one level before any syscall.

The AVR is outside this rule. avr-libc comes in one variant whose
`mem*` are hand-written assembly, small and fast at once; there is no
syscall layer under it, and the AVR build uses avr-libc's own startup.
Nothing that motivates the runtime exists there.

What the rule buys beyond the seven - a C-library call that cannot
link - the AVR gets another way. Its images do take more of avr-libc
than the seven (the number conversions util/print.hpp calls, the
float ones among them), so the library stays on the link line and the
refusal is a list: `avrdx/cmake/avr-refused-libc.rsp` wraps every
function avr-libc's heap and stdio define - `malloc`, `calloc`,
`realloc`, `free`, `strdup`, `strndup` and `atexit` (which allocates),
and every `printf`, `scanf`, stream and character entry - and no image
defines a wrapper. A call to one fails the link as an undefined
`__wrap_<name>`, at the caller's own line. `operator new` needs no
entry: this toolchain's link gives it no definition. `brio check
avrdx` links a reference to every listed name and to `operator new`,
for every package, and expects every one refused.

## The seven functions

- `memcpy` and `memmove` copy words when source and destination share
  their alignment modulo the word: bytes up to a word boundary, then
  blocks of four words, then single words, then the tail. Otherwise they
  copy bytes. `memmove` copies backwards when the destination overlaps
  the source from above, a word a turn.
- `memset` fills bytes up to a word boundary, then the byte replicated
  across blocks of four words, then single words, then the tail; with
  one pointer, the word path is always reachable.
- `memcmp` compares bytes; its callers are rare and short.
- `strlen` and `memchr` scan bytes: the strings brio measures or
  searches at run time are short, and a word-at-a-time scan would buy
  nothing worth its code.
- `abort` spins forever. A breakpoint instruction would be the other
  choice, but on ARMv6-M a breakpoint with no debugger attached
  escalates to HardFault and the frame that got there is lost; a spin
  leaves the whole call stack for a debugger to walk. `abort` does not
  write the panic breadcrumb (kernel.md, section 10): the runtime knows no
  platform, and reaching the breadcrumb needs one.

None of them performs a word access at an address that is not a
multiple of the word: ARMv6-M faults on one and Hazard3 traps, so the
alignment test is part of the contract and not an optimization.

Every loop that moves data is a do-while behind its own test, running to
an end pointer. Written as a plain while loop, it comes out of -Os with
the test at the top and a jump at the bottom - two taken branches a turn
- and a taken branch is dear on the cores that call these functions for
an event: at -Os the Cortex-M4F and the M33 copy an event inline, the
M0+, Hazard3 and the QingKe cores call `memcpy` for one of 52 bytes
(the kernel's queue compiled for each). Measured
on the QingKe V4B at 144 MHz, a 52-byte copy between word-aligned
buffers: 215 cycles in the while shape, 101 in this one, against 91 for
the full newlib's `memcpy`, which moves nine words a turn in 226 bytes
of code where this `memcpy` takes 166. Four words a turn is the point
where a 52-byte event, the largest the census below found, is three
turns and one word.

The file protects itself against GCC's loop recognition, which would
turn the byte loops of `memcpy` and `memset` into calls to `memcpy` and
`memset` - into themselves. Without a guard it does so at -O2 and -O3
(measured on Cortex-M0+ and QingKe: memset calling memset, memcpy calling
memcpy) and not at -Os or -Og - a property of today's optimizer, not a
promise. The guard is an optimization pragma inside the file, so no
build project has to remember a flag; the family check (`brio check`,
`cli/checks/rt_check.sh`) compiles the file with every family's compiler
and flags, at -Os and at -O3, and fails if any of the seven calls one of
the seven.

## Why events need no alignment of their own

The word path only helps if the objects copied are word-aligned. A
census of every event type queued by the 32-bit images found that every
event of eight bytes or more is already aligned to four and a multiple
of four in size - it carries a pointer or a 32-bit value - while the
smaller ones (one to six bytes) are copied inline by the compiler and
never reach `memcpy`. Forcing the queues' storage to word alignment would
cost tens of bytes of RAM per image and speed up nothing, so the kernel's
storage keeps the alignment its event types give it.

## Why a source file in a header-only framework

A function the compiler calls must exist exactly once in the image, as
a real symbol with C linkage. A header cannot provide that: an `inline`
definition is emitted only where the translation unit itself uses it -
a call the compiler generates does not count - and a non-inline one is
defined again by every unit that includes it. So the runtime is the one
`.cpp` under `brio/`, compiled once per image, and the rest of the
framework stays headers. It includes nothing of brio: it is the lowest
stratum (overview.md, "Layering").

Placement: the functions live in `.text` like the rest of the image.
The RP2040's and the RP2350's flash drivers run code from RAM while the
flash is disconnected; that code calls only the bootrom, through
registers, and never a `mem*` (a call from there into flash would
fault). The family check keeps that true: it compiles the flash
driver's fixture and fails if a relocation of `.ram_text` names one of
the seven.

## How it is proven

- On the host, `test/test_rt/` runs the six that copy, fill, compare and
  scan exhaustively against a byte-at-a-time reference: every
  misalignment of source and destination from 0 to 7, every length up to
  several word blocks, every overlap in both directions for `memmove`,
  guard bytes around every destination, the sign `memcmp` gives the
  first differing byte, and `memchr`'s value taken as unsigned char and
  its bound honoured.
- On every compiler, every image of every 32-bit build project links
  with no member of the C library, and the family check verifies the
  self-call guard and the RAM-code rule.
- On silicon the functions depend on the core and the flags, not on a
  peripheral. The cases live in `rt/selftest.hpp`, written once: the host
  suite runs them against the implementations (and against the host's own
  C library, which proves the cases themselves), and letter `t` of every
  32-bit platform suite runs them on the chip against the symbols the
  image links - called through volatile pointers, so the compiler cannot
  expand a copy in place and test itself instead. One chip per core is
  enough for the runtime and one board per build project for the
  integration; the platform suite does both in one run.
