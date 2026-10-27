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
a run-time pointer, `abort` in Debug. `memmove` is referenced by a few
suites, where the compiler expands a move inside an object into a call,
and `memchr` by the benchmark suites that time it; both are provided
because the compiler and the library may emit them anywhere. Nothing
else of the C library is.

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

- `memcpy` copies bytes up to the destination's word boundary, then
  words, then the tail, in one of two forms by what the core allows.
  On the Cortex-M4 and the M33, which load a word from any address, it
  is one leaf for every alignment of its two ends: the head a byte and
  a halfword behind conditions, the source read at its own offset, the
  blocks under 64 bytes four load/store pairs through one register,
  64 bytes or more in turns of four blocks out of line, the tail a
  halfword and a byte (below, "The copy across alignments"). Elsewhere
  the copy asks whether source and destination share their alignment
  modulo the word: when they do, the words go in blocks of four, then
  singly - on the M0+ a block one load-multiple and one store-multiple
  of four registers, 64 bytes or more in turns of four blocks; on a
  RISC-V core four loads and four stores, a block a turn - and when they
  do not, a copy of 16 bytes or more merges aligned source words by
  shifts (below) and a shorter one copies bytes. `memmove`'s forward
  copy is `memcpy`'s body; it copies backwards when the destination
  overlaps the source from above, a word a turn when the two share their
  alignment and bytes when they do not.
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
the copy loads its source words, and its head's and tail's halfword, at
the source's own offset, which the ARMv7-M architecture allows for a
single load or store (ARMv7-M ARM A3.2.1; the RP2350 datasheet
3.7.4.3.2 for the M33) unless CCR.UNALIGN_TRP is set; no startup code
of brio's sets it, and a program that sets it to catch unaligned
accesses catches this copy's too.

Every loop that moves a run of data runs to an end pointer with its
test at the bottom, one branch a turn, and the test that it has a turn
at all is made on the byte count, never on the end pointer (the at most
three head bytes before a word boundary are the exception; the M4's
and the M33's leaf has no such loop, its head and tail each a byte and
a halfword behind a condition, and its block loop runs its count down
below zero and puts it back, one instruction deciding each turn). Written as a plain
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
this `memcpy` takes 194, its copy across alignments 516 more out of
line. Four words a block is the point where a
52-byte event, the largest the census below found, is three blocks and
one word.

### The block on a Thumb core

A Thumb core moves several words in one instruction, and that is where
its block comes from: a load or store multiple of N registers takes
1 + N cycles where a single load or store takes two (DDI 0439B 3.3.1
for the M4, DDI 0432C table 3-1 for the Cortex-M0; the M33's manual
gives no cycle table), and on the M4 a run of STR or STRD is two
cycles a word, measured below, its write buffer holding one store. So
a fill's block is one STM of four registers on the M0+, the M4 and the
M33; a copy's is one LDM and one STM on the M0+, and on the M4 and the
M33 four single loads and one STM, the loads reading the source at any
offset (below) and pipelining to the same 1 + N cycles when it is on a
word; a copy or a fill of 64 bytes or more runs turns of four blocks,
one branch a turn. Measured on the STM32F446 at 180 MHz
(`bench_stm32f4`, letter m: 4096 bytes between two word-aligned SRAM1
buffers, the core's floor two bytes a cycle for a copy and four for a
fill, `x` the wall over that floor):

| shape | memcpy 4096 | memset 4096 |
|---|---|---|
| LDRD/STRD, 16 bytes a turn: GCC's own (the fill with two branches a turn) | 4421 (x 2.15) | 2866 (x 2.79) |
| newlib 4.6.0, `memcpy-armv7m.S` (LDR/STR pairs, 64 bytes a turn) and its C `memset` (STRD, one branch) | 3226 (x 1.57) | 2860 (x 2.79) |
| an aggregate of 64 bytes copied by value (GCC's LDM/STM expansion) | 3107 (x 1.51) | - |
| LDM/STM of four, 16 bytes a turn | 3370 (x 1.64) | 2348 (x 2.29) |
| LDM/STM of four, 64 bytes a turn: the runtime's fill, and the M0+'s copy | 2820 (x 1.37) | 1541 (x 1.50) |
| four LDR and one STM, 64 bytes a turn: the runtime's copy on the M4 and the M33, the one leaf | 2894 (x 1.41) | - |
| LDM/STM of eight (r3..r10), 32 bytes a turn: no Thumb-1 encoding | 2730 (x 1.33) | 1712 (x 1.67) |

The fill's second branch cost it nothing there: four stores of two
cycles bound the turn, which is why only the store-multiple moves it.
The copy's four single loads cost 74 cycles over the load-multiple at
4096 bytes, one a turn and the call its turns are reached by; what they
buy is a copy that asks nothing about its source's alignment, measured
below. At the bench's shorter sizes the runtime takes 34, 65 and 254
cycles to copy 1, 16 and 256 bytes (35, 86 and 341 in LDRD/STRD) and
21, 51 and 161 to fill them (19, 61 and 226).

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

On the M0+ the copy's loops are inline: a block needs four data
registers whatever moves it, so the copy's frame does not grow. The
fill's are not: inline, `memset` would keep its destination and count
live across the four registers, and GCC saves registers at its entry
for every fill; a one-byte fill takes 38 cycles that way on the
STM32F446, 21 this way. So the turns, with the rest of a long fill, are
a function of their own that `memset` reaches for 64 bytes or more (a
tail call on the M4 and the M33, a call on the M0+); on the M4 and the
M33 the copy's turns are out of line for the same reason, its short
path keeping three saved registers (below). The price is code, counted
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
them. The destination is brought to a word boundary, then the words are
copied in the form each core's manual allows:

- On the Cortex-M4 and the M33 there is no second path: one leaf serves
  every alignment. The source is read at its own offset, the core
  splitting each load into aligned bus accesses, on the M4 one cycle
  dearer than an aligned load from a halfword offset and two from a byte
  offset (DDI 0439B 3.3.2; the M33's manual gives no figure, and its
  measured turn is in the table below), and stored as aligned words;
  the head and the tail are a byte and a halfword each, read and written
  at their own addresses behind a condition apiece, no loop; under
  64 bytes a block of 16 is four load/store pairs through one data
  register, so the leaf's prologue saves three registers and no more;
  from 64 bytes the turns of four loads and a store multiple are a
  function of their own, reached by a tail call that costs the one call
  once. What a short copy pays is that fixed cost: a copy of 60 bytes
  takes 95 cycles aligned alike and 125 across on the STM32F446, where a
  copy that asked whether its ends were aligned alike, with its head a
  byte loop and its two paths, took 120 and 165 to 182. Merging aligned
  words by shifts measured slower at every length on both cores.
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
byte and, at its end, the one that holds the last byte its words move,
never a word past the one that holds the source's last byte; with them
it reads up to three bytes on each side of the source. Those bytes lie
in the same aligned word as a byte the copy reads anyway, and on every
core this file runs on, memory is mapped and protected in units of at
least a word, so the read cannot fault; the bytes read are discarded.
The functions are for ordinary memory, SRAM and flash (ARM's Normal
type): a source inside a peripheral's registers would be read in words
it was not asked for, and on the M4 and the M33 an unaligned load from
Device memory faults (ARMv7-M ARM A3.2.1). The host suite proves the
reads' bounds for the form it compiles, four words a turn, and for the
scans below, with a source at either edge of a page whose neighbours
are mapped with no access; the word-a-turn merge of the M0+ and the
QingKe V2 rests on the same arithmetic and on letter t's values, which
cannot see an over-read. The M4's and the M33's leaf reads no byte
outside its source: every load is at the source's own address, of the
bytes asked for.

The fixed cost (up to three head bytes, the setup, a call) decides
where the merging copy starts, at 16 bytes: a copy of 16 bytes is
cheaper than the bytes with no head byte and dearer with three on the
STM32G0, the SAM C21, the CH32V203 and Hazard3, about even taken over
the four destination offsets, and cheaper at every head on the RP2040,
the CH32V303 and the CH32V006. The leaf of the M4 and the M33 has no
such threshold: a copy of any length takes it. Measured with the bench
suites' letter m, the runtime's `memcpy` between static SRAM buffers,
the best of eight runs and the least of three passes, the call
included, against the core's floor (two bytes a cycle);
"across" is the three cases the letter times, the destination one past
a word with the source on a word, the source one past a word with the
destination on one, and the destination one and the source two past a
word:

| core (board, clock) | 60 bytes, aligned alike / across | 4096 bytes, aligned alike / across (x) | the copy across, the turn in the listing |
|---|---|---|---|
| Cortex-M4F (STM32F446RE, 180 MHz) | 95 / 125 | 2894 (1.41) / 4940 to 4952 (2.41) | the leaf's turn of 64 bytes, four blocks of four unaligned loads and a store multiple, one branch: 77 cycles across, 45 aligned alike; under 64 bytes a block of four load/store pairs |
| Cortex-M33 (RP2350, 150 MHz) | 90 / 103 to 105 | 2374 (1.15) / 3395 to 3402 (1.65) | the same instructions: 53 cycles a turn across, 37 aligned alike |
| Cortex-M0+ (STM32G0B1RE, 64 MHz, two wait states) | 204 / 322 to 367 | 2899 (1.41) / 10412 to 10457 (5.10) | LDM of one, two shifts and an OR, STM of one, compare, branch: 10 cycles a word |
| Cortex-M0+ (SAM C21J18A, 48 MHz, two wait states) | 241 / 367 to 425 | 2950 (1.44) / 10457 to 10515 (5.13) | the same: 10 cycles a word |
| Cortex-M0+ (RP2040, 125 MHz) | 176 / 296 to 340 | 2869 (1.40) / 10386 to 10430 (5.09) | the same: 10 cycles a word |
| Hazard3 (RP2350, 150 MHz) | 101 / 156 to 186 | 2868 (1.40) / 5949 to 5990 (2.92) | four loads, eight shifts, four ORs, four stores, two steps and a branch: 23 cycles a turn of 16 bytes |
| QingKe V4B (CH32V203C8, 144 MHz) | 142 / 197 to 236 | 3156 (1.54) / 6242 to 6294 (3.07) | the same 23 instructions: 24 cycles a turn |
| QingKe V4F (CH32V303VC, 144 MHz) | 117 / 174 to 205 | 3137 (1.53) / 6225 to 6263 (3.05) | the same: 24 cycles a turn |
| QingKe V2 (CH32V006K8, 48 MHz, two wait states) | 307 / 532 to 595 | 2048 bytes: 4598 (4.49) / 12460 to 12523 (12.2) | a word a turn, eight instructions: 24 cycles a word, the fetch behind two wait states the rest |

A byte loop moves a byte in 6 cycles on the M4, the M33 and Hazard3,
in 7 on the QingKe V4, in 9 on the M0+ parts and in 15 to 18 on the
QingKe V2, by where the loop lies behind the flash's wait states (the
transports' copy tables, [benchmark.md](benchmark.md): letter u on the
STM32G0, SAM C21, RP2040 and RP2350, letter m on the STM32F4 and the
CH32V203 and CH32V303, letter c on the CH32V00x). On
the M4 and the M33 the unaligned load sets the turn: sixteen of them
at three and a quarter cycles from a byte offset (DDI 0439B 3.3.2: a
byte-aligned load adds two cycles, and a load after a load loses one)
are 52 of the turn's 77 cycles on the M4, the four store multiples 20.
newlib 4.6.0's `memcpy-armv7m.S` reads the source with the same
unaligned loads in 64-byte turns of load/store pairs and does its head
and tail with conditional instructions, but it is not one leaf: it asks
whether the ends are aligned alike, stores unaligned words when only
the source is on a word, and runs a byte loop under 8 bytes across
alignments. It copies 4096 bytes across alignments in 5114 to 5176
cycles on the STM32F446, where the runtime's store multiples bring it
to 4940 to 4952, and 60 bytes in 110 to 116 against the runtime's 125:
newlib's assembly saves no register and counts down with the carry,
where GCC saves and restores three registers around the leaf, some 8
cycles, and tests the count's bits through shifted copies of it.

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
| Hazard3 (RP2350) | 72 to 87 / 128 to 164 / 7198 | 92 to 104 / 165 to 200 / 9250 | seven and eight (the bit-manipulation extension's and-not instruction folds the complement) |
| QingKe V4B (CH32V203C8) | 116 to 137 / 188 to 236 / 9278 | 145 to 163 / 231 to 290 / 11328 | eight and nine |
| QingKe V4F (CH32V303VC) | 95 to 110 / 173 to 206 / 9269 | 119 to 128 / 212 to 244 / 11319 | the same |
| QingKe V2 (CH32V006K8) | 223 to 259 / 394 to 481 / 21610 | 276 to 285 / 471 to 540 / 24693 | the same, behind two wait states |

A byte at a time, the same scans of 4096 bytes take 6 to 9 cycles a
byte on the M4, the M33, Hazard3 and the QingKe V4F (`strlen` the 6 or
7, `memchr` the 7 to 9), 9 and 14 on the QingKe V4B, 8 and 10 on the
M0+ parts and 18 on the QingKe V2, and of 15 bytes 106 to 354 cycles;
the word scan is ahead at 15 bytes on every core but in one case,
`memchr` on the SAM C21 from a start one past a word (279 cycles
against 256). On the STM32F446 newlib's `strlen` and `memchr` read
eight bytes a load and test four bytes an instruction with the DSP
extension's UADD8 and SEL: a third to two fifths less at 4096 (the
table below), from instructions the other cores do not have.

### Against the C library's

The reference the seven functions are judged against is the full
newlib each toolchain ships (4.6.0 for the Arm cores and Hazard3, WCH's
4.5.0 for the QingKe cores), measured with letter m on the same board
and clock as the tables above: newlib's `memcpy`, `strlen` and `memchr`
linked into the suite's image in place of the runtime's three. Its
`memcpy` has two forms. The Thumb-2 cores (the M4, the M33) get an
assembly leaf, `memcpy-armv7m.S`: a test of the two ends once, a
co-aligned path with no head whose 64, 16 and 4-byte blocks are taken
on the count's borrow, and a path across alignments that aligns the
destination unless the source already is, then runs the same blocks
with unaligned loads or unaligned stores. Every other core gets the
portable C `memcpy`: words when the two ends are aligned alike, four a
turn on the M0+ and nine a turn on RISC-V, and a byte loop otherwise.
Its scans are C word scans of the runtime's own shape on every core but
two: the M4's are the DSP assembly above, and Hazard3's use the
bit-manipulation extension's `orc.b`, which marks a word's zero bytes
in one instruction.

| core (board) | | `memcpy` aligned alike, 16 / 60 / 4096 | `memcpy` across, 60 / 4096 | `strlen` 15 / 4096 | `memchr` 15 / 4096 |
|---|---|---|---|---|---|
| Cortex-M4F (STM32F446RE) | runtime | 65 / 95 / 2894 | 125 / 4940 to 4952 | 83 / 8230 | 121 / 11316 |
| | newlib | 38 / 71 / 3059 | 110 to 116 / 5114 to 5176 | 57 / 5552 | 82 / 6709 |
| Cortex-M33 (RP2350) | runtime | 64 / 90 / 2374 | 103 to 105 / 3395 to 3402 | 73 / 8223 | 105 / 11301 |
| | newlib | 39 / 73 / 2394 | 94 to 108 / 3421 to 4444 | 62 / 8230 | 103 / 13345 |
| Cortex-M0+ (STM32G0B1RE) | runtime | 160 / 204 / 2899 | 322 to 367 / 10412 to 10457 | 152 / 12383 | 185 / 15462 |
| | newlib | 148 / 217 / 5503 | 590 / 32878 | 147 / 11871 | 200 / 21601 |
| Cortex-M0+ (SAM C21J18A) | runtime | 200 / 241 / 2950 | 367 to 425 / 10457 to 10515 | 190 / 10377 | 236 / 17551 |
| | newlib | 201 / 277 / 5558 | 622 / 32910 | 192 / 9874 | 245 / 17566 |
| Cortex-M0+ (RP2040) | runtime | 127 / 176 / 2869 | 296 to 340 / 10386 to 10430 | 129 / 10318 | 161 / 13396 |
| | newlib | 128 / 204 / 5483 | 571 / 32859 | 119 / 9801 | 166 / 17487 |
| Hazard3 (RP2350) | runtime | 61 / 101 / 2868 | 156 to 186 / 5949 to 5990 | 72 / 7198 | 92 / 9250 |
| | newlib | 111 / 175 / 2663 | 252 to 260 / 14378 to 14386 | 38 / 4122 | 57 / 7203 |
| QingKe V4B (CH32V203C8) | runtime | 96 / 142 / 3156 | 197 to 236 / 6242 to 6294 | 116 / 9278 | 145 / 11328 |
| | newlib | 87 / 132 / 2714 | 409 / 24625 | 92 / 8257 | 128 / 12357 |
| QingKe V2 (CH32V006K8), 2048 for 4096 | runtime | 184 / 307 / 4598 | 532 to 595 / 12460 to 12523 | 223 / 21610 | 276 / 24693 |
| | newlib | 175 / 280 / 4160 | 988 / 30808 | 169 / 18541 | 252 / 27759 |

Where the runtime is ahead, it is by the shape it was written for: the
copy across alignments, where the portable `memcpy` moves bytes, takes
2.4 (Hazard3) to 3.9 (QingKe V4B) times the runtime's cycles at 4096
and 1.4 to 2.1 times at 60 on every merging core; the M0+'s four
load/store pairs a turn, with no load multiple, take 1.9 times the
runtime's at 4096 aligned alike; `memchr` scans a word a turn where
the portable one scans bytes, 1.3 to 1.4 times the runtime's on the
RP2040 and the STM32G0 at 4096 (on the SAM C21 the two are level,
17551 against 17566). Where newlib is ahead, each gap has its cause in
the two listings:

- The shortest copies on three merging cores, up to 12 cycles (STM32G0
  148 against 160 at 16 bytes, Hazard3 25 against 36 at 1, RP2040 90
  against 95 at 1; the SAM C21 level): the wrapper's call to the copy
  out of line, the cost the merging cores pay for the frame that keeps
  their loop in registers ("The copy across alignments").
- The aligned-alike copy on the QingKe cores at every size, 5 to 17 %
  (V4B 2714 against 3156 at 4096): nine words a turn against four, one
  compare and branch per 36 bytes instead of 16, and the wrapper
  below 64.
- The aligned-alike copy of 16 to 256 bytes on the M33 and the M4, 17
  to 40 cycles on the M33 (39 / 73 / 174 against 64 / 90 / 214 at 16 /
  60 / 256) and 27, 24 and 87 on the M4: newlib's leaf tests the two
  ends once and its co-aligned path has no head, its tail taking only
  the blocks the count holds; the runtime's leaf has one path, so its
  head and tail run on every call, and a conditional instruction
  skipped by an IT block still costs its cycle: 14 instructions in the
  head when the destination is already on a word, 13 in the tail when
  the count is a multiple of 16, two of them taken branches, and the
  three registers GCC saves around the leaf, 8 cycles more. That
  accounts for the M33's gap and for the M4's at 16 and 60; the M4's
  87 at 256 adds the turns' own frame of five registers and a
  remainder this reading does not split. At 4096 the leaf's store
  multiples put it ahead on the M4 (2894 against 3059) and level on
  the M33; across alignments with the destination off a word, where
  newlib stores unaligned words, the leaf is ahead by 23 % on the M33
  (3401 against 4444) and level or ahead elsewhere.
- The scans on Hazard3, `strlen` 43 % less at 4096 and half at 15
  bytes (4122 against 7198, 38 against 72), `memchr` 22 % less (7203
  against 9250): newlib's loop is four instructions, a load, a step,
  `orc.b` and a compare, against the runtime's seven; the zero byte's
  position then falls out of a count-trailing-zeros. The runtime's
  scans are one form for every RISC-V core, and the QingKe cores have
  no bit-manipulation extension; a form under `__riscv_zbb` for the
  cores that have one is not written, a gap this measurement opened.
- `strlen` on the QingKe cores, 11 to 14 % less at 4096 and a fifth to
  a quarter less at 15 bytes (V2 169 against 223): the same seven
  instructions a word, then newlib's tail tests the hit word's three
  bytes in line where the runtime's steps a byte with two taken jumps
  each, dear behind the V2's two wait states.
- `strlen` on the M0+ parts, 4 to 5 % less at 4096: both loops reload
  a constant from the literal pool every word (the scans' table), and
  newlib's is unrolled by two, one taken branch every two words
  against the runtime's one a word.
- The DSP scans on the M4 are the paragraph above.

### What the functions cost in code

Measured with `nm` on the object at -Os with each family's flags:
`memcpy` and `memset` with the functions behind them take 534 bytes on
the M4 and 550 on the M33 (the leaf 166 and 176, its turns 154 and
160), 556 on the M0+, 566 on the QingKe V2 and 814 to 826 on Hazard3
and the QingKe V4, whose copy across alignments is three loops of four
words a turn (516 bytes); `strlen` takes 56 to 74 bytes and `memchr`
88 to 114. An image links only what it calls.

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
  the pages beside it mapped with no access, is copied and scanned at every
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
  cannot run, and the other forms of the copy (the M4's and the M33's
  one leaf, the word a turn of the M0+ and the QingKe V2) are code it
  does not compile, so on those cores letter t is what proves them. Its
  lengths reach 150 bytes: two turns of 64 and the blocks, words and
  bytes after them, at every misalignment of both pointers.
