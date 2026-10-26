# The runtime: what an image takes from the toolchain

This page decides what a brio image on a 32-bit target takes from its
toolchain and why: one library, libgcc, and seven functions of brio's
own in `rt/rt.cpp`, the one source file of the framework (`memcpy`,
`memmove`, `memset`, `memcmp`, `strlen`, `memchr`, `abort`), with no C
library on the link line. It is written for an embedded programmer who
has linked a C program against newlib and wants to know what is
different here, what fails to link on purpose, and what the seven
functions cost. An application does nothing to use the runtime: every
32-bit build project compiles the file into every image. The AVR is
outside this rule and keeps avr-libc; the reason is below.

## What it is for

A brio image on a 32-bit target takes from its toolchain exactly one
library, libgcc, the compiler's own helpers: 64-bit division, count
leading zeros, the Arm case tables. Nothing of the C library enters.
Seven functions that a freestanding C++ program cannot avoid are
brio's own.

They cannot be avoided because the compiler and the library headers
call them, not the application's code:

- GCC turns a copy or a value-initialization of an object above a
  small size into a call to `memcpy` or `memset` (the threshold is low
  at -Os, where a call is smaller than the loads and stores), a
  library algorithm on trivially copyable data into `memmove` or
  `memcmp`, and a loop it recognizes as a copy or a fill into the same
  calls. GCC's manual requires these four of any freestanding
  environment for that reason. In brio's images they copy the event a
  dispatch receives, the bus requests, the packets a USB interrupt
  moves.
- The freestanding libstdc++ brio builds on
  ([overview.md, "Design pillars"](overview.md#design-pillars)) calls
  two more through `std::char_traits<char>`: `length` becomes
  `strlen`, so a `std::string_view` made from a pointer whose contents
  the compiler cannot see needs it, and `find` becomes `memchr`. The
  other traits are the four above.
- libstdc++ compiled with -fno-exceptions turns every throw into a
  call to `abort()`. At -Os the optimizer proves those branches dead;
  at -Og it does not, so a Debug image references `abort()` where a
  Release one does not.

Measured over every image of every 32-bit build project, Release and
Debug, linked with no C library: `memcpy`, `memset` and `memcmp` are
referenced, `strlen` by an image that makes a `std::string_view` from
a run-time pointer, `abort` in Debug. `memmove` and `memchr` are not
referenced today and are provided because the compiler and the
library may emit them. Nothing else of the C library is.

## Why brio's own, and not the C library's

The toolchains ship newlib in two variants, and the link step of each
toolchain follows its own tradition:

- newlib-nano's `mem*` are byte loops: 14 to 36 bytes each, and a
  48-byte event copied one byte at a time. Full newlib's copy words,
  and are ten to twenty-five times larger (up to 378 bytes for one
  function). Neither is right everywhere: a 16 KB part wants the small
  ones, and a copy on the path every event takes wants words.
- Under the C library sits the syscall layer (libgloss): `_write`,
  `_sbrk`, `_exit` and the rest. arm-none-eabi links none of it unless
  a specs file asks, so a `printf` that sneaks in fails the link
  there; both RISC-V toolchains link it always, nano or full, as
  syscalls made with `ecall`, so the mistake links and surfaces as a
  trap when the code runs. A Debug image's `abort()` alone is enough to
  pull that chain in (abort, then raise, the signal machinery, stdio,
  malloc, eight syscalls) wherever nothing else defines `abort`.

Taking the seven functions from brio instead makes all of that one
rule: the same link line on every toolchain, the size and the speed
decided once in one file, and a C-library call that cannot link
anywhere. The functions are small enough that owning them costs less
than choosing, per target, among variants that are each wrong
somewhere.

## The link line

Every 32-bit build project links its images with:

- `-nostartfiles`: the startup code is the project's own (`src/glue/`),
  which also keeps the syscall layer's `crt0.o` out;
- `-nodefaultlibs`: the compiler driver adds no default library at all,
  not the C library, not the syscall layer (they come from the same
  specs rule), not libgcc;
- `rt/rt.cpp` compiled into every image, beside the startup code;
- `-lgcc`, named explicitly and placed after the objects.

No `--specs` file is used: what a specs file would select no longer
reaches the link. The toolchain's headers are untouched (`<cstring>`,
`<cstdint>` and the rest resolve as before; the options above are link
options). An accidental `printf` or `malloc` now fails the link with
the function's own name, one level before any syscall.

The AVR is outside this rule. avr-libc comes in one variant whose
`mem*` are hand-written assembly, small and fast at once; there is no
syscall layer under it, and the AVR build uses avr-libc's own startup.
Nothing that motivates the runtime exists there.

What the rule buys beyond the seven, a C-library call that cannot
link, the AVR gets another way. Its images do take more of avr-libc
than the seven (the number conversions `util/print.hpp` calls, the
float ones among them), so the library stays on the link line and the
refusal is a list: `avrxt/cmake/avr-refused-libc.rsp` wraps every
function avr-libc's heap and stdio define (`malloc`, `calloc`,
`realloc`, `free`, `strdup`, `strndup` and `atexit`, which allocates,
and every `printf`, `scanf`, stream and character entry), and no image
defines a wrapper. A call to one fails the link as an undefined
`__wrap_<name>`, at the caller's own line. `operator new` needs no
entry: this toolchain's link gives it no definition. `brio check
avrxt` links a reference to every listed name and to `operator new`,
for every package, and expects every one refused.

## The seven functions

- `memcpy` and `memmove` copy bytes up to the destination's word
  boundary, then words, then the tail. When source and destination
  share their alignment modulo the word, the words go in blocks of
  four, then singly: on a Thumb core (the M0+, the M4, the M33) a block
  is one load-multiple and one store-multiple of four registers, and 64
  bytes or more run in turns of four blocks; on a RISC-V core a block is
  four loads and four stores, a block a turn. When they do not, a copy
  of 8 bytes or more on the M4 and the M33, 16 or more elsewhere, runs
  across alignments (below), and a shorter one copies bytes. `memmove`
  copies backwards when the destination overlaps the source from above,
  a word a turn when the two share their alignment and bytes when they
  do not.
- `memset` fills bytes up to a word boundary, then the byte replicated
  across blocks of four words, then single words, then the tail; with
  one pointer, the word path is always reachable. On a Thumb core a
  fill of 64 bytes or more runs turns of four store-multiples, out of
  line (below).
- `memcmp` compares bytes; its callers are rare and short.
- `strlen` and `memchr` scan bytes up to a word boundary, then aligned
  words, then the bytes of the word that holds the end (below).
- `abort` spins forever. A breakpoint instruction would be the other
  choice, but on ARMv6-M a breakpoint with no debugger attached
  escalates to a HardFault and the frame that got there is lost; a
  spin leaves the whole call stack for a debugger to walk. `abort`
  does not write the panic breadcrumb
  ([kernel.md, "Failures: overflow and panic"](kernel.md#10-failures-overflow-and-panic-kernelpanichpp)):
  the runtime knows no platform, and reaching the breadcrumb needs
  one.

Outside the Cortex-M4 and the M33, none of them performs a word access
at an address that is not a multiple of the word: ARMv6-M faults on
one, Hazard3 requires natural alignment of every access (the RP2350
datasheet 3.8.9), and the QingKe V2 and V4 manuals list a misaligned
load or store as an exception (2.1, table 2-1). On the M4 and the M33
the copy across alignments loads its source at the source's own
offset, which the ARMv7-M architecture allows for a single load
(ARMv7-M ARM A3.2.1; the RP2350 datasheet 3.7.4.3.2 for the M33) unless
CCR.UNALIGN_TRP is set; no startup code of brio's sets it, and a
program that sets it to catch unaligned accesses catches this one too.

Every loop that moves a run of data runs to an end pointer with its
test at the bottom, one branch a turn, and the test that it has a turn
at all is made on the byte count, never on the end pointer (the at most
three head bytes before a word boundary are the exception). Written as a plain
while loop, or with the same pointer test before the loop and at its
foot (GCC at -Os merges the two), it comes out with the test at the
top and a jump at the bottom, two branches a turn, and a taken branch
is dear on the cores that call these functions for an event: at -Os
the Cortex-M4F and the M33 copy an event inline, while the M0+,
Hazard3 and the QingKe cores call `memcpy` for one of 52 bytes (the
kernel's queue compiled for each). Measured on the QingKe V4B at
144 MHz, a 52-byte copy between word-aligned buffers: 215 cycles in
the while shape, 101 in this one, against 91 for the full newlib's
`memcpy`, which moves nine words a turn in 226 bytes of code where
this `memcpy` takes 164. Four words a block is the point where a
52-byte event, the largest the census below found, is three blocks and
one word.

### The block on a Thumb core

A Thumb core moves several words in one instruction, and that is where
its block comes from: a load or store multiple of N registers takes
1 + N cycles where a single load or store takes two (DDI 0439B 3.3.1
for the M4, DDI 0432C table 3-1 for the Cortex-M0; the M33's manual
gives no cycle table), and on the M4 a run of STR or STRD is two
cycles a word, measured below, its write buffer holding one store. So
on the M0+, the M4 and the M33 a block is one LDM and one STM of four
registers, and a copy or a fill of 64 bytes or more runs turns of four
blocks, one branch a turn. Measured on the STM32F446 at 180 MHz
(`bench_stm32f4`, letter m: 4096 bytes between two word-aligned SRAM1
buffers, the core's floor two bytes a cycle for a copy and four for a
fill, `x` the wall over that floor):

| shape | memcpy 4096 | memset 4096 |
|---|---|---|
| LDRD/STRD, 16 bytes a turn: GCC's own (the fill with two branches a turn) | 4421 (x 2.15) | 2866 (x 2.79) |
| newlib 4.6.0, `memcpy-armv7m.S` (LDR/STR pairs, 64 bytes a turn) and its C `memset` (STRD, one branch) | 3226 (x 1.57) | 2860 (x 2.79) |
| an aggregate of 64 bytes copied by value (GCC's LDM/STM expansion) | 3107 (x 1.51) | - |
| LDM/STM of four, 16 bytes a turn | 3370 (x 1.64) | 2348 (x 2.29) |
| LDM/STM of four, 64 bytes a turn: the runtime | 2820 (x 1.37) | 1541 (x 1.50) |
| LDM/STM of eight (r3..r10), 32 bytes a turn: no Thumb-1 encoding | 2730 (x 1.33) | 1712 (x 1.67) |

The fill's second branch cost it nothing there: four stores of two
cycles bound the turn, which is why only the store-multiple moves it.
At the bench's shorter sizes the runtime takes 36, 80 and 240 cycles
to copy 1, 16 and 256 bytes (35, 86 and 341 in LDRD/STRD) and 21, 51
and 161 to fill them (19, 61 and 226).

GCC is not asked for these instructions, because it does not produce
them well. Copied as an aggregate, a block becomes LDM/STM pairs
through four fixed registers with both addresses recomputed every
block (the table's third row), and on RISC-V at -Os the same aggregate
becomes a call to `memcpy`, inside `memcpy`. For a fill it forms no
STM at all: its peephole wants four ascending registers, and the
register allocator does not hand it four for one value. So the block
loops are written in the instructions themselves, under `__thumb__`,
the one place the file speaks an instruction set: the Thumb-1 subset
every Thumb core decodes, with low registers, writeback, four fixed
data registers declared clobbered, and never a register list built
from operands, which would follow the allocator's order. An interrupt
arriving during one of them does not wait for it: the M4 and the M33
continue the transfer after the handler (the M33's manual lists LDM
and STM among its exception-continuable instructions), and the M0+
abandons it and starts it again, which loads or stores the same words
a second time.

The copy's loops are inline: a block needs four data registers
whatever moves it, so the copy's frame does not grow. The fill's are
not: inline, `memset` would keep its destination and count live across
the four registers, and GCC saves registers at its entry for every
fill; a one-byte fill takes 38 cycles that way on the STM32F446, 20
this way. So the turns, with the rest of a long fill, are a function
of their own that `memset` reaches for 64 bytes or more (a tail call
on the M4 and the M33, a call on the M0+). The price is code, counted
below with the copy across alignments.

The file protects itself against GCC's loop recognition, which would
turn the byte loops of `memcpy` and `memset` into calls to `memcpy`
and `memset`, into themselves. Without a guard it does so at -O2 and
-O3 (measured on the Cortex-M0+ and the QingKe: `memset` calling
`memset`, `memcpy` calling `memcpy`) and not at -Os or -Og, a property
of today's optimizer and not a promise. The guard is an optimization
pragma inside the file, so no build project has to remember a flag;
the family check (`brio check`, `cli/checks/rt_check.sh`) compiles the
file with every family's compiler and flags, at -Os and at -O3, and
fails if any of the seven calls one of the seven.

### The copy across alignments

A copy into a transport's transmit ring starts wherever the ring's head
is, so its two ends share their alignment one time in four; the other
three are the common case, not the rare one, and this is the copy for
them. The destination is brought to a word boundary by bytes, then the
words are copied in the form each core's manual allows:

- On the Cortex-M4 and the M33 the source is read with single loads at
  its own offset, four a turn, and stored as aligned words; the core
  splits each load into aligned bus accesses, one cycle dearer than an
  aligned load from a halfword offset and two from a byte offset (DDI
  0439B 3.3.2). Merging aligned words by shifts measured slower at every
  length on both cores.
- Everywhere else the source is read as aligned words, the word holding
  its next byte first, and each word stored is merged from two of them
  by shifts. The shift counts are constants, one copy of the loop for
  each of the three offsets: with the counts in registers the loop
  spills to the stack on the Cortex-M0+, whose Thumb-1 instructions
  reach eight registers, and on the QingKe V2 (RV32E, sixteen). The
  M0+ and the QingKe V2 merge a word a turn, four a turn spilling again
  on the first and costing some 300 bytes on the second, whose
  smallest part (the CH32V003) has 15 KB for an image; the other RISC-V
  cores merge four words a turn.

The merging copy reads the aligned word that holds the source's first
byte and the one that holds its last, and with them up to three bytes
on each side of the source. Those bytes lie in the same aligned word as
a byte the copy reads anyway, and on every core this file runs on,
memory is mapped and protected in units of at least a word, so the read
cannot fault; the bytes read are discarded. The host suite proves it
with a source at either edge of a page that has no page mapped beside
it, and the scans below read the same way.

The fixed cost (up to three head bytes, the setup, on the M0+ and the
RISC-V cores a call) decides where the copy across alignments starts:
at 8 bytes on the M4 and the M33, where it is as cheap as the bytes
already, and at 16 elsewhere, where a copy of 16 bytes is cheaper with
no head byte and dearer with three, about even taken over the four
destination offsets. Measured with the bench suites' letter m, the
runtime's `memcpy` between static SRAM buffers, the best of eight
runs, the call included, against the core's floor (two bytes a cycle);
"across" is the three cases the letter times, the destination one past
a word with the source on a word, the source one past a word with the
destination on one, and the destination one and the source two past a
word:

| core (board, clock) | 60 bytes, aligned alike / across | 4096 bytes, aligned alike / across (x) | the copy across, the turn in the listing |
|---|---|---|---|
| Cortex-M4F (STM32F446RE, 180 MHz) | 120 / 165 to 182 | 2820 (1.37) / 6465 to 6490 (3.16) | four unaligned loads, two STRD, three steps and a branch: 25 cycles a turn of 16 bytes |
| Cortex-M33 (RP2350, 150 MHz) | 99 / 127 to 141 | 2295 (1.12) / 4661 to 4684 (2.28) | the same instructions: 18 cycles a turn |
| Cortex-M0+ (STM32G0B1RE, 64 MHz, two wait states) | 204 / 322 to 367 | 2899 (1.41) / 10412 to 10457 (5.10) | LDM of one, two shifts and an OR, STM of one, compare, branch: 10 cycles a word |
| Cortex-M0+ (SAM C21J18A, 48 MHz, two wait states) | 241 / 367 to 425 | 2950 (1.44) / 10457 to 10515 (5.13) | the same: 10 cycles a word |
| Cortex-M0+ (RP2040, 125 MHz) | 176 / 296 to 340 | 2869 (1.40) / 10386 to 10430 (5.09) | the same: 10 cycles a word |
| Hazard3 (RP2350, 150 MHz) | 101 / 156 to 186 | 2868 (1.40) / 5949 to 5990 (2.92) | four loads, eight shifts, four ORs, four stores, two steps and a branch: 23 cycles a turn of 16 bytes |
| QingKe V4B (CH32V203C8, 144 MHz) | 142 / 197 to 236 | 3156 (1.54) / 6242 to 6294 (3.07) | the same 23 instructions: 24 cycles a turn |
| QingKe V4F (CH32V303VC, 144 MHz) | 117 / 174 to 205 | 3137 (1.53) / 6225 to 6263 (3.05) | the same: 24 cycles a turn |
| QingKe V2 (CH32V006K8, 48 MHz, two wait states) | 307 / 532 to 595 | 2048 bytes: 4598 (4.49) / 12460 to 12523 (12.2) | a word a turn, eight instructions: 24 cycles a word, the fetch behind two wait states the rest |

A byte loop moves a byte in 6 cycles on the M4, the M33 and Hazard3,
in 7 on the QingKe V4, in 9 on the M0+ parts and in 15 on the QingKe
V2 (the transports' copy tables, [benchmark.md](benchmark.md), letter
m). On
the M4 and the M33 the unaligned load sets the turn: four of them at
four cycles from a byte offset are 16 of its 25 cycles on the M4.
newlib 4.6.0's `memcpy-armv7m.S`, which aligns the destination and
then reads the source with the same unaligned loads in 64-byte turns,
copies 4096 bytes across alignments in 5114 to 5176 cycles on the
STM32F446 and 60 bytes in 110 to 116: its long turn saves a fifth at
4096, and one leaf function with no co-alignment test and a branchless
head and tail is what it saves at 60.

### The scans

`strlen` and `memchr` read bytes up to a word boundary, then aligned
words, each tested for a zero byte with the carry trick: a word w holds
a zero byte exactly when (w - 0x01010101) & ~w & 0x80808080 is not
zero. It needs no multiply, which the QingKe V2 of the CH32V003 lacks;
`memchr` first takes an exclusive or with its byte replicated across
the word (shifts and ORs in the source, a multiply where the compiler
has one to fold them into). Then the bytes of the word that hit. The
last word read is the one that holds the string's last byte, the
argument of the copy above. Measured with letter m on strings of bytes
above 0x7F in SRAM, a word-aligned start and one past a word, `memchr`
asked for the NUL within 64 bytes or, for the longest, within n + 1:

| core | `strlen` 15 / 50 / 4096 | `memchr` 15 / 50 / 4096 | loop in the listing, a word |
|---|---|---|---|
| Cortex-M4F (STM32F446RE) | 83 to 93 / 148 to 178 / 8230 | 121 to 139 / 211 to 254 / 11316 | six instructions and nine |
| Cortex-M33 (RP2350) | 73 to 79 / 139 to 161 / 8223 | 105 to 114 / 196 to 226 / 11301 | the same |
| Cortex-M0+ (STM32G0B1RE) | 152 to 185 / 253 to 302 / 12383 | 185 to 204 / 311 to 352 / 15462 | seven and ten, one constant reloaded from the literal pool every word |
| Cortex-M0+ (SAM C21J18A) | 190 to 207 / 273 to 308 / 10377 | 236 to 279 / 380 to 442 / 17551 | the same |
| Cortex-M0+ (RP2040) | 129 to 144 / 212 to 245 / 10318 | 161 to 176 / 269 to 307 / 13396 | the same |
| Hazard3 (RP2350) | 72 to 87 / 128 to 164 / 7198 | 92 to 104 / 165 to 200 / 9250 | seven and eight (Zbb's ANDN folds the complement) |
| QingKe V4B (CH32V203C8) | 116 to 137 / 188 to 236 / 9278 | 145 to 163 / 231 to 290 / 11328 | eight and nine |
| QingKe V4F (CH32V303VC) | 95 to 110 / 173 to 206 / 9269 | 119 to 128 / 212 to 244 / 11319 | the same |
| QingKe V2 (CH32V006K8) | 223 to 259 / 394 to 481 / 21610 | 276 to 285 / 471 to 540 / 24693 | the same, behind two wait states |

A byte at a time, the same scans of 4096 bytes take 6 to 9 cycles a
byte on the M4, the M33, Hazard3 and the QingKe V4F (`strlen` the 6 or
7, `memchr` the 7 to 9), 9 and 14 on the QingKe V4B, 8 and 10 on the
M0+ parts and 18 on the QingKe V2, and of 15 bytes 106 to 354 cycles;
the word scan is ahead at 15 bytes on every core but in one case,
`memchr` on the SAM C21 from a start one past a word (279 cycles
against 256). On the STM32F446 newlib 4.6.0's `strlen` and `memchr`,
which read eight bytes a load and test four bytes an instruction with
the DSP extension's UADD8 and SEL, take 57 to 78 and 82 to 147 cycles
for 15 bytes and 5552 and 6709 for 4096: a third to two fifths less,
from instructions the other cores do not have.

### What the functions cost in code

Measured with `nm` on the object at -Os with each family's flags:
`memcpy` and `memset` with the functions behind them take 484 bytes on
the M4 and the M33, 556 on the M0+, 566 on the QingKe V2 and 814 to
826 on Hazard3 and the QingKe V4, whose copy across alignments is
three loops of four words a turn (516 bytes); `strlen` takes 56 to 74
bytes and `memchr` 88 to 114. An image links only what it calls.

## Why events need no alignment of their own

The block path, the fastest, wants both ends aligned alike, and an
event copied between word-aligned storage has them so when its type is
word-aligned. A census of every event type queued by the 32-bit images found that
every event of eight bytes or more is already aligned to four and a
multiple of four in size, because it carries a pointer or a 32-bit
value, while the smaller ones (one to six bytes) are copied inline by
the compiler and never reach `memcpy`. Forcing the queues' storage to
word alignment would cost tens of bytes of RAM per image and speed up
nothing, so the kernel's storage keeps the alignment its event types
give it.

## Why a source file in a header-only framework

A function the compiler calls must exist exactly once in the image,
as a real symbol with C linkage. A header cannot provide that: an
`inline` definition is emitted only where the translation unit itself
uses it (a call the compiler generates does not count), and a
non-inline one is defined again by every unit that includes it. So
the runtime is the one `.cpp` under `brio/`, compiled once per image,
and the rest of the framework stays headers. It includes nothing of
brio: it is the lowest stratum
([overview.md, "Layering"](overview.md#layering-the-strata)).

Placement: the functions live in `.text` like the rest of the image.
The RP2040's and the RP2350's flash drivers run code from RAM while
the flash is disconnected; that code calls only the bootrom, through
registers, and never a `mem*` (a call from there into flash would
fault). The family check keeps that true: it compiles the flash
driver's fixture and fails if a relocation of `.ram_text` names one of
the seven.

## How it is proven

- On the host, `test/test_rt/` runs the six that copy, fill, compare
  and scan exhaustively against a byte-at-a-time reference: every
  misalignment of source and destination from 0 to 7, every length up
  to several word blocks, every overlap in both directions for
  `memmove`, guard bytes around every destination, the sign `memcmp`
  gives the first differing byte, and `memchr`'s value taken as
  unsigned char and its bound honoured. A source, or a string, whose
  first byte is a page's first or whose last byte is a page's last,
  the pages beside it unmapped, is copied and scanned at every
  destination misalignment and every length to 130 bytes: a read one
  word past the source or before it faults there instead of passing.
- On every compiler, every image of every 32-bit build project links
  with no member of the C library, and the family check verifies the
  self-call guard and the RAM-code rule.
- On silicon the functions depend on the core and the flags, not on a
  peripheral. The cases live in `rt/selftest.hpp`, written once: the
  host suite runs them against the implementations (and against the
  host's own C library, which proves the cases themselves), and letter
  `t` of every 32-bit platform suite runs them on the chip against the
  symbols the image links, called through volatile pointers, so the
  compiler cannot expand a copy in place and test itself instead. One
  chip per core is enough for the runtime and one board per build
  project for the integration; the platform suite does both in one
  run.
- The host compiles what the RISC-V cores with 32 registers run: the
  block of four loads and four stores and the copy across alignments
  merging four words a turn. The Thumb block loops are instructions it
  cannot run, and the other forms of the copy across alignments (the
  M4's and the M33's unaligned loads, the word a turn of the M0+ and
  the QingKe V2) are code it does not compile, so on those cores letter
  t is what proves them. Its lengths reach 150 bytes: two turns of 64
  and the blocks, words and bytes after them, at every misalignment of
  both pointers.
