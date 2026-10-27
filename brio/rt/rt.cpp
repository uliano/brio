/*
 * rt.cpp
 *
 * The runtime: the seven functions a freestanding C++ image cannot avoid,
 * and the one source file of the framework (design/runtime.md).
 *
 *   memcpy, memmove, memset, memcmp  what GCC calls on its own - for a
 *                                    copy or a value-initialization of
 *                                    an object above a small size, for a
 *                                    library algorithm on trivially
 *                                    copyable data, for a loop it
 *                                    recognizes as a copy or a fill;
 *   strlen, memchr                   what the freestanding libstdc++
 *                                    brio builds on calls besides:
 *                                    std::char_traits<char>'s length and
 *                                    find, so a std::string_view made
 *                                    from a pointer, or searched;
 *   abort                            what libstdc++ calls on a throw
 *                                    path under -fno-exceptions, which
 *                                    -Og leaves alive.
 *
 * COMPILED INTO EVERY 32-BIT IMAGE, beside its crt, and linked with
 * -nostartfiles -nodefaultlibs and libgcc: no C library and no syscall
 * layer reach the link, so an accidental printf or malloc fails there
 * with its own name. The AVR does not compile this file: avr-libc's mem*
 * are hand-written assembly, small and fast at once.
 *
 * THE WORD PATH. A copy moves words once its destination is on a word
 * boundary: bytes up to that boundary, then words, then the tail. A copy
 * into a transport's ring meets every alignment of its two ends (the
 * ring's head is wherever the last run left it), so the ends aligned
 * alike are one case in four, and the copy takes the form each core's
 * manual allows for the other three.
 *
 * THE ONE LEAF OF THE CORTEX-M4 AND THE M33. These cores load a word or a
 * halfword from any address (ARMv7-M ARM A3.2.1, the RP2350 datasheet
 * 3.7.4.3.2 for the M33; LDM, LDRD and the rest still fault, and so does
 * LDR itself if CCR.UNALIGN_TRP is set - no crt of brio's sets it, and a
 * program that sets it to find stray unaligned accesses finds this copy's
 * too, as it finds the compiler's own): the core splits the access into
 * aligned bus accesses, one or two cycles dearer than an aligned one (DDI
 * 0439B 3.3.2). So there the copy never asks whether its two ends are
 * aligned alike: it brings the destination to a word boundary and reads
 * the source at whatever offset it has, with the same instructions at
 * every alignment. What a short copy pays is then one fixed cost, and the
 * leaf keeps it to a prologue that saves three registers: no loop in the
 * head or the tail (a byte and a halfword each, conditional), the blocks
 * under 64 bytes through one data register, and the long turns out of
 * line, their call paid once for 64 bytes or more. Measured on the
 * STM32F446 (letter m of its bench suite), a copy of 60 bytes takes 95
 * cycles with the two ends aligned alike and 125 across, where the copy
 * that asked took 120 and 165 to 182, and newlib's assembly, which saves
 * no register, 110 to 116. Merging aligned words by shifts, the other
 * cores' form, measured slower on both cores at every length.
 *
 * THE COPY ACROSS ALIGNMENTS BY MERGING, everywhere else, where an
 * unaligned word access is a fault or a trap - ARMv6-M faults on one,
 * Hazard3 "requires natural alignment of all accesses" (the RP2350
 * datasheet 3.8.9, MCAUSE codes 4 and 6), and the QingKe V2 and V4
 * manuals list load and store address misalignment as exceptions (2.1,
 * table 2-1, codes 4 and 6). When source and destination share their
 * alignment the words go in blocks of four (below), then singly; when
 * they do not, for a copy of rt_shift_min bytes or more - a shorter one
 * stays a byte loop, the fixed cost being more than its bytes - the
 * source is read as ALIGNED words, the word that holds its next byte
 * first, and each word stored is merged from two of them by shifts
 * (little-endian: the low word shifted right by 8k bits, the high one
 * left by 32 - 8k, k the source's offset in its word, 1 to 3) - the two
 * counts in registers on a RISC-V core with 32 registers, one loop for
 * the three offsets, and constants where the registers are few, one loop
 * per offset (at rt_merge_words). There no function here performs a word
 * access at an address that is not a multiple of the word.
 *
 * WHY READING WHOLE WORDS IS SOUND. The merging copy reads the aligned
 * word that holds the source's first byte and, at its end, the one that
 * holds the last byte its words move - never a word past the one that
 * holds the source's last byte: bytes before the first and after the
 * last are read with them. They lie in the same aligned word as a byte
 * the copy reads anyway, and on every core this file runs on memory is
 * mapped, protected and faulted in units of at least a word (a memory's
 * end, an MPU or PMP region), so they are readable wherever that byte
 * is; their values are discarded. strlen and memchr read the same way,
 * the aligned word that holds the string's last byte and nothing past
 * it. These functions are for ordinary memory - SRAM and flash, ARM's
 * Normal type: a source inside a peripheral's registers is read in words
 * it was not asked for, and on the M4 and the M33 an unaligned load from
 * Device memory faults (ARMv7-M ARM A3.2.1). A sanitizer that tracks
 * objects byte by byte would see those reads; the host suite's arenas
 * carry the slack around every source (test/test_rt/), and a source that
 * ends at the last byte of a page, with no readable page after it, is one
 * of its cases. The one leaf of the M4 and the M33 reads no byte outside
 * its source: every load is at the source's own address, of the bytes
 * asked for.
 *
 * THE SCANS. strlen and memchr read bytes up to a word boundary, then
 * aligned words, testing each for a zero byte with the carry trick -
 * (w - 0x01010101) & ~w & 0x80808080, no multiply, which the CH32V003's
 * QingKe V2 has none of - memchr after an exclusive or with its byte
 * replicated across the word, then the bytes of the word that hit. Two
 * cores mark the bytes instead: a RISC-V core with the Zbb extension
 * (__riscv_zbb: Hazard3 has it, the QingKe cores have not) with orc.b,
 * which writes every byte of its result as 0xFF where the byte read is
 * not zero and 0x00 where it is, and the Cortex-M4 and M33
 * (__ARM_FEATURE_SIMD32) with UADD8 and SEL, which do the same in two.
 * With the marks the lowest zero byte's position is the count of the
 * marks' trailing ones divided by eight (ctz of the complement), so no
 * byte loop follows the hit, and none precedes the words either: the
 * scan starts at the word that holds its first byte, read whole, with
 * the bytes before that byte forced to 0xFF. The M4 and the M33 read
 * their words in pairs at addresses that are multiples of eight (LDRD),
 * the second word's marks laid over the first's by SEL, one branch for
 * eight bytes. What a scan with marks reads beyond the bytes asked for
 * lies within the aligned word, or the aligned pair, that holds the
 * first byte and the one that holds the last (for memchr, the first and
 * the words before the bound's own): never across a boundary of eight
 * bytes, so never into another page or another region of the memory
 * protection unit.
 *
 * THE BLOCK ON A THUMB CORE. The Cortex-M0+, M4 and M33 move several
 * words in one instruction: a load or store multiple of N registers takes
 * 1 + N cycles, where a single load or store takes two (and on the M4 a
 * run of STR or STRD is two cycles a word, its write buffer holding one
 * store). There a fill's block is one STM of four registers, and a copy's
 * one LDM and one STM on the M0+ - on the M4 and the M33 four single
 * loads, which read the source at any offset and pipeline to the same
 * 1 + N from an aligned one, and one STM; a copy or a fill of 64 bytes or
 * more runs turns of four blocks with one branch a turn. The pairs the
 * leaf copies its blocks with under 64 bytes are written out too, for the
 * registers' sake (at rt_copy_rest). GCC is not asked for these instructions, because it does
 * not produce them well: an aggregate copied by value is expanded into
 * LDM/STM pairs that recompute both addresses every block, and becomes a
 * call to memcpy on RISC-V at -Os; and for a fill it forms no STM at all,
 * its peephole wanting four ascending registers that the register
 * allocator never hands it for one value (design/runtime.md has the
 * numbers of each shape). So these loops are the one place the file speaks
 * the instruction set, under __thumb__: the Thumb-1 subset every Thumb
 * core decodes - low registers, writeback - in instructions that GCC's
 * divided syntax (its inline assembly on ARMv6-M) and unified syntax read
 * alike. A RISC-V core has no such instruction; there a block is loads
 * and stores - eight words a turn in the co-aligned copy of a core with
 * 32 registers, then a block of four in line, four a turn everywhere else
 * (at rt_copy_blocks).
 *
 * THE SHAPE OF THE LOOPS. Every loop that moves data runs to an end
 * pointer with its test at the bottom: one branch a turn. The test that a
 * loop has a turn at all is made on the byte count, never on the end
 * pointer - with the same pointer test before the loop and at its foot,
 * GCC at -Os merges the two and rotates the loop to its test at the top
 * and a jump at the bottom, two branches a turn, which on the QingKe V4B
 * cost a 52-byte copy twice the cycles (design/runtime.md has the
 * numbers).
 *
 * THE HOT LOOPS ON A WORD BOUNDARY. On the RISC-V cores every loop that
 * moves words is entered through an alignment directive, so that its
 * first instruction lies on a word: the QingKe V4 pays a cycle for a
 * taken branch to a 32-bit instruction at an odd halfword, the QingKe V2
 * a fetch for every word of code the loop spans, and GCC at -Os aligns no
 * loop whatever it is told (the measurements and the mechanism at
 * rt_loop_on_word).
 *
 * THE SELF-CALL TRAP. GCC recognizes a byte loop that copies or fills and
 * replaces it with a call to memcpy or memset - here, a call to the very
 * function being defined. Measured without the guard: at -O2 memset
 * becomes a call to memset, at -O3 memcpy a call to memcpy; at -Os and -Og
 * it does not happen today, which is a property of the optimizer and not
 * a promise. The pragma below turns that recognition off for this file
 * alone, so no build project has to remember a flag; the family check
 * verifies on every compiler that none of these functions calls one of
 * the seven.
 *
 * On the host, test/test_rt/ includes this file with BRIO_RT_HOST_TEST
 * defined: the implementations are compiled and tested as the QingKe
 * targets compile them, and the C-linkage definitions - which would
 * replace the host C library's own - are left out. The Thumb block loops
 * and the Zbb scans are instructions the host cannot run; letter t of
 * every platform suite runs the cases on the chip (design/runtime.md,
 * "How it is proven").
 */

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#if defined(__riscv_zbb)
#include <riscv_bitmanip.h>
#elif defined(__ARM_FEATURE_SIMD32)
#include <arm_acle.h>
#endif

#pragma GCC push_options
#pragma GCC optimize("no-tree-loop-distribute-patterns")

namespace {

// A word as memory seen through a pointer that may alias anything: the
// copies move the bytes of objects of every type, so the word accesses
// must not be subject to the type-based aliasing rules.
using RtWord = uint32_t __attribute__((__may_alias__));

constexpr size_t rt_word_bytes = sizeof(RtWord);
constexpr uintptr_t rt_word_mask = rt_word_bytes - 1u;

uintptr_t rt_address(const void* p) {
    return reinterpret_cast<uintptr_t>(p);
}

// A block of the word path: four words.
constexpr size_t rt_block_bytes = 4u * rt_word_bytes;

// THE HOT LOOPS ON A WORD BOUNDARY (RISC-V). Every loop below that moves
// words is entered through this: a `.balign 4` directive, so that the
// loop's first instruction lies on a word. What a core pays without it,
// by its manual and by letter m of the bench suites: the QingKe V4 pays
// a cycle for a taken branch to a 32-bit instruction at an odd halfword
// (the CH32V203's co-aligned copy of 4096 bytes, 3413 cycles with its
// block loop's first load at an odd halfword against 3156 on a word; the
// merging copy 6532 against 6279; a loop whose first instruction is a
// 16-bit one pays nothing), the QingKe V2 pays a fetch, three cycles
// behind its flash's two wait states, for every word of code the loop
// spans (the CH32V006's strlen of 4096 bytes, 21610 cycles with its loop
// spanning six words against 18538 spanning five; its co-aligned copy of
// 2048 bytes 4598 against 4221), and Hazard3 documents a cycle for a
// branch to a 32-bit instruction that is not word-aligned (the RP2350
// datasheet 3.8.7.4), which its branch predictor hides in a loop (3.8.7.10,
// the two fetch paths stitched when the branch is predicted; measured
// level either way). GCC at -Os aligns no loop, whatever -falign-loops or
// the align-loops pragma asks (measured with both: its alignment pass
// stands down when optimizing for size), so the directive is asked of
// the assembler, as an asm statement that names the loop's variables as
// its operands: the register copies GCC makes to enter a loop then come
// before it, and the loops below advance the caller's pointers first and
// run on copies of their own, so that nothing is left to copy between the
// directive and the loop's first instruction - a placement read in the
// listings of both compilers (the upstream GCC 16.2 and WCH's 15.2) and
// checked by no tool (design/runtime.md). The linker keeps the alignment
// through its relaxation. Three arities, because neither RISC-V compiler
// accepts a parameter pack expanded in an asm's operands; on the other
// cores the statement is empty (the Thumb blocks are instructions of
// their own, and the Arm cores are not measured for it).
template <typename A>
[[gnu::always_inline]] inline void rt_loop_on_word(A& a) {
#if defined(__riscv)
    __asm__ volatile(".balign 4" : "+r"(a));
#else
    static_cast<void>(a);
#endif
}
template <typename A, typename B>
[[gnu::always_inline]] inline void rt_loop_on_word(A& a, B& b) {
#if defined(__riscv)
    __asm__ volatile(".balign 4" : "+r"(a), "+r"(b));
#else
    static_cast<void>(a);
    static_cast<void>(b);
#endif
}
template <typename A, typename B, typename C>
[[gnu::always_inline]] inline void rt_loop_on_word(A& a, B& b, C& c) {
#if defined(__riscv)
    __asm__ volatile(".balign 4" : "+r"(a), "+r"(b), "+r"(c));
#else
    static_cast<void>(a);
    static_cast<void>(b);
    static_cast<void>(c);
#endif
}

// The fill from its first word boundary to its end: whole blocks, then
// words, then bytes - the byte being the word's low one. On a Thumb core
// what is left after the turns; on another, all of it. The block loop
// runs on a copy of d, advanced past the blocks first (rt_loop_on_word).
[[gnu::always_inline]] inline void rt_fill_rest(unsigned char* d, const RtWord w, size_t n) {
    const size_t span = n & ~(rt_block_bytes - 1u);
    if (span != 0u) {
        unsigned char* dd = d;
        d += span;
        rt_loop_on_word(dd);
        do {
            reinterpret_cast<RtWord*>(dd)[0] = w;
            reinterpret_cast<RtWord*>(dd)[1] = w;
            reinterpret_cast<RtWord*>(dd)[2] = w;
            reinterpret_cast<RtWord*>(dd)[3] = w;
            dd += rt_block_bytes;
        } while (dd != d);
    }
    n &= rt_block_bytes - 1u;
    if (n >= rt_word_bytes) {
        unsigned char* const words = d + (n & ~rt_word_mask);
        do {
            *reinterpret_cast<RtWord*>(d) = w;
            d += rt_word_bytes;
        } while (d != words);
    }
    n &= rt_word_mask;
    if (n != 0u) {
        unsigned char* const last = d + n;
        do {
            *d++ = static_cast<unsigned char>(w);
        } while (d != last);
    }
}

#if defined(__thumb__)

// THE THUMB BLOCK LOOPS (the file's header says why they are written out).
// The data registers r3..r6 are fixed and declared clobbered, never
// operands: a register list built from operands would follow the register
// allocator's order, which the assembler takes as a list out of order and
// a fill's four copies of one value may share. The operands are low
// registers ("l"), as every Thumb-1 load and store multiple needs.
constexpr size_t rt_turn_bytes = 4u * rt_block_bytes;

#if !defined(__ARM_FEATURE_UNALIGNED)

// The whole blocks of n from s to d, both advanced: turns of 64 bytes,
// then up to three blocks. Inline in the copy: a block wants four data
// registers whatever instruction moves it, so the copy's frame is the one
// the scalar block needs. The Cortex-M0+'s; the M4 and the M33 move their
// blocks in the one leaf below, which reads the source at any offset.
[[gnu::always_inline]] inline void rt_copy_blocks(unsigned char*& d, const unsigned char*& s, const size_t n) {
    if (n >= rt_turn_bytes) {
        const unsigned char* const turns = d + (n & ~(rt_turn_bytes - 1u));
        __asm__ volatile(
            "1:\n\t"
            "ldmia %1!, {r3, r4, r5, r6}\n\t"
            "stmia %0!, {r3, r4, r5, r6}\n\t"
            "ldmia %1!, {r3, r4, r5, r6}\n\t"
            "stmia %0!, {r3, r4, r5, r6}\n\t"
            "ldmia %1!, {r3, r4, r5, r6}\n\t"
            "stmia %0!, {r3, r4, r5, r6}\n\t"
            "ldmia %1!, {r3, r4, r5, r6}\n\t"
            "stmia %0!, {r3, r4, r5, r6}\n\t"
            "cmp %0, %2\n\t"
            "bne 1b"
            : "+l"(d), "+l"(s)
            : "l"(turns)
            : "r3", "r4", "r5", "r6", "cc", "memory");
    }
    if ((n & (rt_turn_bytes - rt_block_bytes)) != 0u) {
        const unsigned char* const blocks = d + (n & (rt_turn_bytes - rt_block_bytes));
        __asm__ volatile(
            "1:\n\t"
            "ldmia %1!, {r3, r4, r5, r6}\n\t"
            "stmia %0!, {r3, r4, r5, r6}\n\t"
            "cmp %0, %2\n\t"
            "bne 1b"
            : "+l"(d), "+l"(s)
            : "l"(blocks)
            : "r3", "r4", "r5", "r6", "cc", "memory");
    }
}

#endif

// A fill of 64 bytes or more from its first word boundary: the turns, then
// the rest; returns dst. OUT OF LINE, unlike the copy's loops: inline,
// memset would keep its destination and count live across r3..r6, and GCC
// saves registers for them at memset's entry, on every fill - a one-byte
// fill takes 38 cycles on the STM32F446 that way, 21 this way. A fill this
// long pays one call. The parameters come in this order so that memset's
// short path keeps its byte in r1 and needs no register saved at all.
[[gnu::noinline]] void* rt_fill_turns(unsigned char* d, const RtWord w, const size_t n, void* const dst) {
    const unsigned char* const turns = d + (n & ~(rt_turn_bytes - 1u));
    __asm__ volatile(
        "mov r3, %2\n\t"
        "mov r4, %2\n\t"
        "mov r5, %2\n\t"
        "mov r6, %2\n"
        "1:\n\t"
        "stmia %0!, {r3, r4, r5, r6}\n\t"
        "stmia %0!, {r3, r4, r5, r6}\n\t"
        "stmia %0!, {r3, r4, r5, r6}\n\t"
        "stmia %0!, {r3, r4, r5, r6}\n\t"
        "cmp %0, %1\n\t"
        "bne 1b"
        : "+l"(d)
        : "l"(turns), "l"(w)
        : "r3", "r4", "r5", "r6", "cc", "memory");
    rt_fill_rest(d, w, n & (rt_turn_bytes - 1u));
    return dst;
}

// The fill past its first word boundary; returns dst.
[[gnu::always_inline]] inline void* rt_fill_words(void* const dst, unsigned char* const d, const RtWord w,
                                                  const size_t n) {
    if (n >= rt_turn_bytes) {
        return rt_fill_turns(d, w, n, dst);
    }
    rt_fill_rest(d, w, n);
    return dst;
}

#else

// The co-aligned copy's turn: eight words where the registers allow,
// four on RV32E (the reasons below).
#if defined(__riscv_e)
constexpr size_t rt_copy_turn_bytes = rt_block_bytes;
#else
constexpr size_t rt_copy_turn_bytes = 2u * rt_block_bytes;
#endif

// The whole turns of n from s to d, both advanced: loads and stores,
// rt_copy_turn_bytes a turn, then one block of four in line where the
// turn is two blocks. Eight words a turn on a RISC-V core with 32
// registers: with four, the branch a turn put this copy behind newlib's
// nine words a turn (the CH32V203 at 4096 bytes, 3156 cycles with four a
// turn, 2649 with eight, newlib 2714; Hazard3 2866, 2485 and 2663), and
// the block in line keeps a copy of 16 to 31 bytes past the turns out of
// the word loop (60 bytes: 136 against 142 on the CH32V203, 97 against
// 103 on Hazard3). Not on RV32E: eight words and the pointers do not fit
// its sixteen registers (read in the listing, two of the words spilled
// and reloaded every turn), so the QingKe V2 keeps a block a turn. The
// loop runs on copies of d and s, advanced past the turns first
// (rt_loop_on_word).
[[gnu::always_inline]] inline void rt_copy_blocks(unsigned char*& d, const unsigned char*& s, const size_t n) {
    const size_t span = n & ~(rt_copy_turn_bytes - 1u);
    if (span != 0u) {
        unsigned char* dd = d;
        const unsigned char* ss = s;
        d += span;
        s += span;
        rt_loop_on_word(dd, ss);
        do {
            const RtWord w0 = reinterpret_cast<const RtWord*>(ss)[0];
            const RtWord w1 = reinterpret_cast<const RtWord*>(ss)[1];
            const RtWord w2 = reinterpret_cast<const RtWord*>(ss)[2];
            const RtWord w3 = reinterpret_cast<const RtWord*>(ss)[3];
            if constexpr (rt_copy_turn_bytes == 2u * rt_block_bytes) {
                const RtWord w4 = reinterpret_cast<const RtWord*>(ss)[4];
                const RtWord w5 = reinterpret_cast<const RtWord*>(ss)[5];
                const RtWord w6 = reinterpret_cast<const RtWord*>(ss)[6];
                const RtWord w7 = reinterpret_cast<const RtWord*>(ss)[7];
                reinterpret_cast<RtWord*>(dd)[0] = w0;
                reinterpret_cast<RtWord*>(dd)[1] = w1;
                reinterpret_cast<RtWord*>(dd)[2] = w2;
                reinterpret_cast<RtWord*>(dd)[3] = w3;
                reinterpret_cast<RtWord*>(dd)[4] = w4;
                reinterpret_cast<RtWord*>(dd)[5] = w5;
                reinterpret_cast<RtWord*>(dd)[6] = w6;
                reinterpret_cast<RtWord*>(dd)[7] = w7;
            } else {
                reinterpret_cast<RtWord*>(dd)[0] = w0;
                reinterpret_cast<RtWord*>(dd)[1] = w1;
                reinterpret_cast<RtWord*>(dd)[2] = w2;
                reinterpret_cast<RtWord*>(dd)[3] = w3;
            }
            dd += rt_copy_turn_bytes;
            ss += rt_copy_turn_bytes;
        } while (dd != d);
    }
    if constexpr (rt_copy_turn_bytes != rt_block_bytes) {
        if ((n & rt_block_bytes) != 0u) {
            const RtWord w0 = reinterpret_cast<const RtWord*>(s)[0];
            const RtWord w1 = reinterpret_cast<const RtWord*>(s)[1];
            const RtWord w2 = reinterpret_cast<const RtWord*>(s)[2];
            const RtWord w3 = reinterpret_cast<const RtWord*>(s)[3];
            reinterpret_cast<RtWord*>(d)[0] = w0;
            reinterpret_cast<RtWord*>(d)[1] = w1;
            reinterpret_cast<RtWord*>(d)[2] = w2;
            reinterpret_cast<RtWord*>(d)[3] = w3;
            d += rt_block_bytes;
            s += rt_block_bytes;
        }
    }
}

// The fill past its first word boundary; returns dst.
[[gnu::always_inline]] inline void* rt_fill_words(void* const dst, unsigned char* const d, const RtWord w,
                                                  const size_t n) {
    rt_fill_rest(d, w, n);
    return dst;
}

#endif

// THE COPY. Two forms, by what the core allows (the file's header).
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "the copy and the scans read words little-endian");

#if defined(__ARM_FEATURE_UNALIGNED)

// THE ONE LEAF of the Cortex-M4 and the M33 (the file's header says why).
// The destination is brought to a word boundary and the source is read at
// whatever offset it has, so no path asks whether the two ends are aligned
// alike, and what a short copy pays is one fixed cost: a prologue that
// saves three registers, a head and a tail with no loop (a byte and a
// halfword each, conditional), the blocks under 64 bytes through one data
// register, and the long turns out of line, their call paid once for 64
// bytes or more.

// A word or a halfword at any address: the compiler emits a plain LDR,
// LDRH, STR or STRH, which the core splits into aligned bus accesses.
using RtWordUnaligned = uint32_t __attribute__((__may_alias__, __aligned__(1)));
using RtHalfUnaligned = uint16_t __attribute__((__may_alias__, __aligned__(1)));

// The tail of a copy, the bits of n below the word: a halfword, then a
// byte, each behind a condition and not a loop.
[[gnu::always_inline]] inline void rt_copy_tail(unsigned char* d, const unsigned char* s, const size_t n) {
    if ((n & 2u) != 0u) {
        *reinterpret_cast<RtHalfUnaligned*>(d) = *reinterpret_cast<const RtHalfUnaligned*>(s);
        d += 2;
        s += 2;
    }
    if ((n & 1u) != 0u) {
        *d = *s;
    }
}

// The rest of a copy to a word-aligned d, under 64 bytes: blocks of 16 as
// four load/store pairs through one data register, then two words, one
// word and the tail by the bits of n. The pairs are written in the
// instructions: the Thumb-2 load and store with an immediate offset, 16-bit
// encodings, which read the source at its own offset and store aligned
// words; left to GCC, the loop ran on an end pointer and a remaining count
// recomputed every turn, two registers more, and memcpy saved four
// registers at its entry instead of three (read in the listing). The
// count runs down below zero and is put back afterwards, so one
// instruction a turn decides the loop.
[[gnu::always_inline]] inline void rt_copy_rest(unsigned char* d, const unsigned char* s, size_t n) {
    if (n >= rt_block_bytes) {
        n -= rt_block_bytes;
        __asm__ volatile(
            "1:\n\t"
            "ldr r3, [%1]\n\t"
            "str r3, [%0]\n\t"
            "ldr r3, [%1, #4]\n\t"
            "str r3, [%0, #4]\n\t"
            "ldr r3, [%1, #8]\n\t"
            "str r3, [%0, #8]\n\t"
            "ldr r3, [%1, #12]\n\t"
            "str r3, [%0, #12]\n\t"
            "adds %1, #16\n\t"
            "adds %0, #16\n\t"
            "subs %2, #16\n\t"
            "bhs 1b"
            : "+l"(d), "+l"(s), "+l"(n)
            :
            : "r3", "cc", "memory");
        n += rt_block_bytes;
    }
    if ((n & 8u) != 0u) {
        reinterpret_cast<RtWord*>(d)[0] = reinterpret_cast<const RtWordUnaligned*>(s)[0];
        reinterpret_cast<RtWord*>(d)[1] = reinterpret_cast<const RtWordUnaligned*>(s)[1];
        d += 8;
        s += 8;
    }
    if ((n & 4u) != 0u) {
        *reinterpret_cast<RtWord*>(d) = *reinterpret_cast<const RtWordUnaligned*>(s);
        d += 4;
        s += 4;
    }
    rt_copy_tail(d, s, n);
}

// A copy of 64 bytes or more past the head, out of line: turns of four
// blocks, a block four loads at the source's own offset and one store
// multiple of the four words, one branch a turn; then the rest; returns
// dst. Out of line for memset's reason: inline, the four data registers
// would be saved on every copy, where a copy this long pays one call. The
// parameters come in this order so memcpy's tail call is two moves and no
// stack. A load multiple would want the source on a word, and asking whether
// it is would make two paths of one: four single loads from an aligned
// source pipeline to the same 1 + N cycles, and the store multiple keeps
// the stores at 1 + N where a run of STR costs two a word (DDI 0439B
// 3.3.1; the fixed cost the leaf gives back for that is in
// design/runtime.md).
[[gnu::noinline, gnu::aligned(4)]] void* rt_copy_turns(unsigned char* d, const unsigned char* s, const size_t n,
                                                       void* const dst) {
    const unsigned char* const turns = d + (n & ~(rt_turn_bytes - 1u));
    __asm__ volatile(
        "1:\n\t"
        "ldr r3, [%1]\n\t"
        "ldr r4, [%1, #4]\n\t"
        "ldr r5, [%1, #8]\n\t"
        "ldr r6, [%1, #12]\n\t"
        "stmia %0!, {r3, r4, r5, r6}\n\t"
        "ldr r3, [%1, #16]\n\t"
        "ldr r4, [%1, #20]\n\t"
        "ldr r5, [%1, #24]\n\t"
        "ldr r6, [%1, #28]\n\t"
        "stmia %0!, {r3, r4, r5, r6}\n\t"
        "ldr r3, [%1, #32]\n\t"
        "ldr r4, [%1, #36]\n\t"
        "ldr r5, [%1, #40]\n\t"
        "ldr r6, [%1, #44]\n\t"
        "stmia %0!, {r3, r4, r5, r6}\n\t"
        "ldr r3, [%1, #48]\n\t"
        "ldr r4, [%1, #52]\n\t"
        "ldr r5, [%1, #56]\n\t"
        "ldr r6, [%1, #60]\n\t"
        "stmia %0!, {r3, r4, r5, r6}\n\t"
        "adds %1, #64\n\t"
        "cmp %0, %2\n\t"
        "bne 1b"
        : "+l"(d), "+l"(s)
        : "l"(turns)
        : "r3", "r4", "r5", "r6", "cc", "memory");
    rt_copy_rest(d, s, n & (rt_turn_bytes - 1u));
    return dst;
}

// The copy, inline in memcpy: a copy under four bytes is its tail; one of
// four or more brings the destination to a word boundary by a byte and a
// halfword, then takes the turns or the rest. The head's two conditions
// and the tail's halfword compile to IT blocks, the tail's byte to a short
// branch (read in the listing).
[[gnu::always_inline]] inline void* rt_memcpy(void* const dst, const void* const src, size_t n) {
    auto* d = static_cast<unsigned char*>(dst);
    const auto* s = static_cast<const unsigned char*>(src);
    if (n >= rt_word_bytes) {
        const size_t head = (0u - rt_address(d)) & rt_word_mask;
        if ((head & 1u) != 0u) {
            *d++ = *s++;
        }
        if ((head & 2u) != 0u) {
            *reinterpret_cast<RtHalfUnaligned*>(d) = *reinterpret_cast<const RtHalfUnaligned*>(s);
            d += 2;
            s += 2;
        }
        n -= head;
        if (n >= rt_turn_bytes) {
            return rt_copy_turns(d, s, n, dst);
        }
        rt_copy_rest(d, s, n);
        return dst;
    }
    rt_copy_tail(d, s, n);
    return dst;
}

#else

// THE COPY ACROSS ALIGNMENTS by merging (the file's header says why).
// rt_shift_min is the shortest copy that takes it: where its fixed cost
// (up to three head bytes, the setup, the call) is paid back against the
// byte loop, the destination's offset in its word taken as equally likely.
// Measured with the bench suites' letter m and its copy table: a copy of
// 16 bytes is cheaper than the bytes with no head byte and dearer with
// three on the STM32G0, the SAM C21, the CH32V203 and Hazard3, about even
// taken over the four, and cheaper at every head on the RP2040, the
// CH32V303 and the CH32V006. It leaves at least one word after the head
// bytes.
constexpr size_t rt_shift_min = 16;

// The words of n (four or more) to a word-aligned d out of aligned source
// words, `lo` the one already read, `from` the next: each word stored is
// lo >> right | hi << left, right = 8K and left = 32 - 8K for K the
// source's offset in its word. Where the registers are few the two counts
// are constants and the loop has one instance per offset: with the counts
// in registers the loop spills to the stack on the Cortex-M0+ (whose
// Thumb-1 instructions reach eight registers) and on the QingKe V2
// (RV32E, sixteen: read in the listing, a count reloaded from the stack
// every word), and a constant shift is no instruction more than a
// variable one. On a RISC-V core with 32 registers the counts are two
// registers and the loop is one: a shift by a register costs what a shift
// by an immediate does (the RP2350 datasheet 3.8.7.1 for Hazard3; the
// QingKe V4 manual gives no cycle table, and the CH32V203 measured 6279
// cycles for 4096 bytes against 6294 with the constants), and the three
// instances cost 516 bytes on Hazard3 and the QingKe V4B where the one
// costs 238. Thumb-1 runs a word a turn (four a turn spill again, measured
// slower on the STM32G0), and so does RV32E, the QingKe V2 of the CH32V00x
// parts: there four a turn cost some 300 bytes, which the 15 KB image of
// the CH32V003 does not have to spare, for 10 to 16 per cent at 60 bytes
// on the CH32V006. The other RISC-V cores run four a turn, then single
// words; the block loop runs on a copy of d, advanced past the blocks
// first (rt_loop_on_word).
[[gnu::always_inline]] inline void rt_merge_words(unsigned char*& d, const RtWord* from, RtWord lo, size_t n,
                                                  const unsigned right, const unsigned left) {
#if !defined(__thumb__) && !defined(__riscv_e)
    const size_t span = n & ~(rt_block_bytes - 1u);
    if (span != 0u) {
        unsigned char* dd = d;
        d += span;
        rt_loop_on_word(dd, from, lo);
        do {
            const RtWord w0 = from[0];
            const RtWord w1 = from[1];
            const RtWord w2 = from[2];
            const RtWord w3 = from[3];
            reinterpret_cast<RtWord*>(dd)[0] = (lo >> right) | (w0 << left);
            reinterpret_cast<RtWord*>(dd)[1] = (w0 >> right) | (w1 << left);
            reinterpret_cast<RtWord*>(dd)[2] = (w1 >> right) | (w2 << left);
            reinterpret_cast<RtWord*>(dd)[3] = (w2 >> right) | (w3 << left);
            lo = w3;
            from += 4;
            dd += rt_block_bytes;
        } while (dd != d);
    }
    n &= rt_block_bytes - 1u;
    if (n < rt_word_bytes) {
        return;
    }
#endif
    unsigned char* const words = d + (n & ~rt_word_mask);
    do {
        const RtWord hi = *from++;
        *reinterpret_cast<RtWord*>(d) = (lo >> right) | (hi << left);
        lo = hi;
        d += rt_word_bytes;
    } while (d != words);
}

// The words of n (four or more) from an s that is not word-aligned to a
// word-aligned d, both advanced. The first read is the aligned word that
// holds s's first byte, the last the one that holds the last byte these
// words move (the file's header: why that is sound). The shift counts are
// constants or registers by the core's register count (rt_merge_words).
[[gnu::always_inline]] inline void rt_copy_words_across(unsigned char*& d, const unsigned char*& s, size_t n) {
    const uintptr_t k = rt_address(s) & rt_word_mask;
    const RtWord* const from = reinterpret_cast<const RtWord*>(s - k);
    const RtWord lo = from[0];
    s += n & ~rt_word_mask;
#if defined(__thumb__) || defined(__riscv_e)
    if (k == 1u) {
        rt_merge_words(d, from + 1, lo, n, 8u, 24u);
    } else if (k == 2u) {
        rt_merge_words(d, from + 1, lo, n, 16u, 16u);
    } else {
        rt_merge_words(d, from + 1, lo, n, 24u, 8u);
    }
#else
    const unsigned right = 8u * static_cast<unsigned>(k);
    rt_merge_words(d, from + 1, lo, n, right, 32u - right);
#endif
}


static_assert(rt_shift_min >= rt_word_bytes + rt_word_mask, "a word past the head bytes");


// A copy across alignments of rt_shift_min bytes or more: bytes up to d's
// word boundary, the words, the tail. Out of line, so its registers are
// its own and the co-aligned copy's frame does not grow (a tail call on
// RISC-V, a call on Thumb-1). The head is counted: the test on the count,
// one branch a byte (the file's header), where the pointer test made two -
// 14 cycles on a copy of 16 bytes to a destination one past a word, on the
// CH32V203.
[[gnu::noinline, gnu::aligned(4)]] void rt_copy_across(unsigned char* d, const unsigned char* s, size_t n) {
    const size_t head = (0u - rt_address(d)) & rt_word_mask;
    if (head != 0u) {
        unsigned char* const boundary = d + head;
        n -= head;
        do {
            *d++ = *s++;
        } while (d != boundary);
    }
    rt_copy_words_across(d, s, n);
    n &= rt_word_mask;
    if (n != 0u) {
        unsigned char* const last = d + n;
        do {
            *d++ = *s++;
        } while (d != last);
    }
}

// Forward copy: correct for disjoint ranges and for a destination below
// an overlapping source. On a word boundary, like rt_copy_across, strlen
// and memchr, for the short loops at its start (its word loops are entered
// through rt_loop_on_word, which says what an odd halfword costs). OUT OF
// LINE, memcpy a frame around its call (two pushes, a call and a return:
// some 13 cycles on the Cortex-M0+ and 6 to 8 on the RISC-V cores, counted
// in the listings), because the forms that spend less were read and
// measured worse: inlined into memcpy, which must keep dst for its return,
// GCC on the M0+ saved eight registers and kept the count on the stack
// (within a few cycles of this form either way in letter m); handed dst to
// return itself, so that memcpy is a tail call, the M0+ spilled it and
// made no tail call (GCC makes none on Thumb-1).
[[gnu::noinline, gnu::aligned(4)]] void rt_copy_forward(unsigned char* d, const unsigned char* s, size_t n) {
    if (n >= rt_word_bytes && ((rt_address(d) ^ rt_address(s)) & rt_word_mask) == 0u) {
        while ((rt_address(d) & rt_word_mask) != 0u) {
            *d++ = *s++;
            --n;
        }
        rt_copy_blocks(d, s, n);
        n &= rt_block_bytes - 1u;
        if (n >= rt_word_bytes) {
            unsigned char* const words = d + (n & ~rt_word_mask);
            do {
                *reinterpret_cast<RtWord*>(d) = *reinterpret_cast<const RtWord*>(s);
                d += rt_word_bytes;
                s += rt_word_bytes;
            } while (d != words);
        }
        n &= rt_word_mask;
    } else if (n >= rt_shift_min) {
        rt_copy_across(d, s, n);
        return;
    }
    // The bytes: a copy too short for the word path, or the tail.
    if (n != 0u) {
        unsigned char* const last = d + n;
        do {
            *d++ = *s++;
        } while (d != last);
    }
}

[[gnu::always_inline]] inline void* rt_memcpy(void* dst, const void* src, size_t n) {
    rt_copy_forward(static_cast<unsigned char*>(dst), static_cast<const unsigned char*>(src), n);
    return dst;
}

#endif

// Backward copy, from the end: correct for a destination above an
// overlapping source. The same shape, mirrored, a word a turn, and bytes
// across alignments: memmove's callers are a few suites and GCC's own
// expansion of a short move, none on a measured path, and the merging
// form written backwards would be a mirror of three more loops, not a
// share - not written until a caller pays for it.
void rt_copy_backward(unsigned char* d, const unsigned char* s, size_t n) {
    d += n;
    s += n;
    if (n >= rt_word_bytes && ((rt_address(d) ^ rt_address(s)) & rt_word_mask) == 0u) {
        while ((rt_address(d) & rt_word_mask) != 0u) {
            *--d = *--s;
            --n;
        }
        if (n >= rt_word_bytes) {
            unsigned char* const words = d - (n & ~rt_word_mask);
            do {
                d -= rt_word_bytes;
                s -= rt_word_bytes;
                *reinterpret_cast<RtWord*>(d) = *reinterpret_cast<const RtWord*>(s);
            } while (d != words);
        }
        n &= rt_word_mask;
    }
    if (n != 0u) {
        unsigned char* const first = d - n;
        do {
            *--d = *--s;
        } while (d != first);
    }
}

void* rt_memmove(void* dst, const void* src, size_t n) {
    auto* d = static_cast<unsigned char*>(dst);
    const auto* s = static_cast<const unsigned char*>(src);
    // Compared as integers: the pointers may belong to different objects,
    // where a pointer comparison has no meaning in the language.
    if (rt_address(d) <= rt_address(s) || rt_address(d) >= rt_address(s) + n) {
        // memcpy's own body, inline here and not a call to memcpy: memcpy's
        // parameters are restrict-qualified, and a forward copy of two
        // overlapping ranges must keep every store behind the loads above
        // it, which that qualifier would let the compiler reorder. On the
        // merge cores this is the frame around rt_copy_forward, as before.
        return rt_memcpy(dst, src, n);
    }
    rt_copy_backward(d, s, n);
    return dst;
}

void* rt_memset(void* dst, int c, size_t n) {
    auto* d = static_cast<unsigned char*>(dst);
    const auto v = static_cast<unsigned char>(c);
    if (n >= rt_word_bytes) {
        while ((rt_address(d) & rt_word_mask) != 0u) {
            *d++ = v;
            --n;
        }
        const RtWord w = static_cast<RtWord>(v) * static_cast<RtWord>(0x01010101u);
        return rt_fill_words(dst, d, w, n);
    }
    if (n != 0u) {
        unsigned char* const last = d + n;
        do {
            *d++ = v;
        } while (d != last);
    }
    return dst;
}

// The sign of the first differing byte, both read as unsigned char.
int rt_memcmp(const void* a, const void* b, size_t n) {
    const auto* x = static_cast<const unsigned char*>(a);
    const auto* y = static_cast<const unsigned char*>(b);
    for (; n != 0u; --n, ++x, ++y) {
        if (*x != *y) {
            return static_cast<int>(*x) - static_cast<int>(*y);
        }
    }
    return 0;
}

// A word holding a zero byte, by the carry trick: a zero byte borrows
// through its top bit in w - 0x01010101 and has that bit clear in w, so
// the lowest zero byte always raises its 0x80. A byte above it may raise
// its own falsely, from the borrow; the scans look at the bytes of the
// word that hits, never at which bits it raised.
[[gnu::always_inline]] inline bool rt_has_zero(const RtWord w) {
    return ((w - RtWord{0x01010101u}) & ~w & RtWord{0x80808080u}) != 0u;
}

#if defined(__riscv_zbb) || defined(__ARM_FEATURE_SIMD32)
#define RT_MARKS 1
// THE SCANS WITH MARKS (the file's header: the Zbb extension's orc.b, the
// DSP extension's UADD8 and SEL). The marks of a word: 0xFF for every
// byte that is not zero, 0x00 for every zero byte; a word with no zero
// byte marks as all ones.
[[gnu::always_inline]] inline RtWord rt_byte_marks(const RtWord w) {
#if defined(__riscv_zbb)
    return __riscv_orc_b_32(w);
#else
    // UADD8 adds 0xFF to every byte and sets that byte's GE flag where
    // the sum carried, where the byte is not zero; SEL then takes, byte
    // by byte, 0xFF where the flag is set and the word's own byte, a
    // zero, where it is not.
    static_cast<void>(__uadd8(w, ~RtWord{0}));
    return __sel(~RtWord{0}, w);
#endif
}

constexpr RtWord rt_no_zero_marks = ~RtWord{0};

// The index of the lowest zero byte of a word that has one, from its
// marks: the complement's trailing zeros are eight per byte below it.
[[gnu::always_inline]] inline size_t rt_lowest_zero(const RtWord marks) {
    return static_cast<size_t>(__builtin_ctz(~marks)) >> 3;
}

// A word with the bytes below an address in it forced to 0xFF: a scan
// with marks starts at the word that holds its first byte, read whole,
// and the bytes before that byte can hold no zero and match no pattern:
// the word ORed with the mask of its bytes below the offset, all ones
// shifted up by the offset's bytes and complemented.
[[gnu::always_inline]] inline RtWord rt_head_masked(const RtWord w, const size_t offset) {
    return w | ~(~RtWord{0} << (8u * offset));
}
#endif

#if defined(__ARM_FEATURE_SIMD32)
// THE SCANS BY PAIRS OF WORDS (Cortex-M4, M33). The marks of the second
// word laid over the first's: SEL takes the first's marks where the
// second's byte is not zero and the second's byte, a zero, where it is,
// so the pair marks as all ones exactly when neither word holds a zero -
// one compare and one branch for eight bytes, which the two words' loads
// share in one LDRD. The pair is read at an address that is a multiple
// of eight, so it never crosses a boundary of 32 bytes: the memory
// protection unit's smallest region, and every page's.
[[gnu::always_inline]] inline RtWord rt_marks_over(const RtWord first, const RtWord w) {
    static_cast<void>(__uadd8(w, ~RtWord{0}));
    return __sel(first, w);
}

constexpr uintptr_t rt_pair_mask = 2u * rt_word_bytes - 1u;

// A pair with the bytes below an offset in it (0 to 7) forced to 0xFF:
// the bytes below the offset in its word, as a mask ORed into the word
// the offset falls in (ORN on these cores); the word below it, if any,
// whole. GCC makes an IT block of the choice.
[[gnu::always_inline]] inline void rt_pair_head_masked(RtWord& lo, RtWord& hi, const size_t offset) {
    const RtWord low_bytes = ~(~RtWord{0} << (8u * (offset & rt_word_mask)));
    lo |= low_bytes;
    if ((offset & rt_word_bytes) != 0u) {
        hi |= low_bytes;
        lo = ~RtWord{0};
    }
}
#endif

// The length of a zero-terminated string. With marks: the word (the pair
// on the DSP cores) that holds the first byte, read whole from its
// boundary with the bytes before the first forced non-zero, then words
// (pairs) until one holds a zero byte, whose index the marks give.
// Otherwise: bytes up to a word boundary, then aligned words until one
// holds a zero byte, then its bytes.
size_t rt_strlen(const char* s) {
#if defined(__ARM_FEATURE_SIMD32)
    const RtWord* w = reinterpret_cast<const RtWord*>(rt_address(s) & ~rt_pair_mask);
    RtWord lo = w[0];
    RtWord hi = w[1];
    rt_pair_head_masked(lo, hi, rt_address(s) & rt_pair_mask);
    RtWord first = rt_byte_marks(lo);
    RtWord both = rt_marks_over(first, hi);
    if (both == rt_no_zero_marks) {
        do {
            w += 2;
            lo = w[0];
            hi = w[1];
            first = rt_byte_marks(lo);
            both = rt_marks_over(first, hi);
        } while (both == rt_no_zero_marks);
    }
    // The pair that hit: the zero in its first word, or in its second,
    // whose marks the pair's are when the first has none.
    const char* q = reinterpret_cast<const char*>(w);
    if (first == rt_no_zero_marks) {
        q += rt_word_bytes;
        first = both;
    }
    return static_cast<size_t>(q - s) + rt_lowest_zero(first);
#elif defined(RT_MARKS)
    const RtWord* w = reinterpret_cast<const RtWord*>(rt_address(s) & ~rt_word_mask);
    // Written from w, stepped past the word read, for the loop GCC makes
    // of it: the load, the step, orc.b, the compare; keeping the hit
    // word's address costs a fifth instruction a word.
    RtWord m = rt_byte_marks(rt_head_masked(*w++, rt_address(s) & rt_word_mask));
    if (m == rt_no_zero_marks) {
        rt_loop_on_word(w);
        do {
            m = rt_byte_marks(*w++);
        } while (m == rt_no_zero_marks);
    }
    return static_cast<size_t>(reinterpret_cast<const char*>(w) - s) - rt_word_bytes + rt_lowest_zero(m);
#else
    const char* p = s;
    while ((rt_address(p) & rt_word_mask) != 0u) {
        if (*p == '\0') {
            return static_cast<size_t>(p - s);
        }
        ++p;
    }
    const RtWord* w = reinterpret_cast<const RtWord*>(p);
    RtWord x;
    rt_loop_on_word(w);
    do {
        x = *w++;
    } while (!rt_has_zero(x));
#if !defined(__thumb__) || defined(__thumb2__)
    // The tail starts from w as the loop left it: without this barrier GCC
    // keeps a copy of w from before the step inside the loop, one
    // instruction a word (the CH32V203 at 4096 bytes: 9276 cycles against
    // 8255, newlib's loop of the same seven instructions 8257). Not on
    // Thumb-1: there the copy is the loop's load-multiple's base, and the
    // barrier turns it into a load and an add, seven instructions either
    // way; 8 bytes more of layout and the same cycles (measured on the
    // STM32G0B1, 12383 against 12384 at 4096 bytes).
    __asm__ volatile("" : "+r"(w));
#endif
    p = reinterpret_cast<const char*>(w) - rt_word_bytes;
    while (*p != '\0') {
        ++p;
    }
    return static_cast<size_t>(p - s);
#endif
}

// The first byte equal to (unsigned char)c within n bytes, or nothing.
// With marks: the word (the pair on the DSP cores) that holds the first
// byte, read whole with the bytes before it forced to no match, then
// words (pairs) up to the one that holds the bound, each xored with the
// byte replicated, until one holds a zero byte - whose index the marks
// give, within the bound because every word scanned lies before the
// bound's own - or the words run out, then the bytes to the bound.
// Otherwise: bytes up to a word boundary, then the words, then the bytes
// of the word that hit, or the bound's.
void* rt_memchr(const void* s, int c, size_t n) {
    const auto* p = static_cast<const unsigned char*>(s);
    const auto v = static_cast<unsigned char>(c);
#if defined(__ARM_FEATURE_SIMD32)
    const unsigned char* const last = p + n;
    const RtWord* w = reinterpret_cast<const RtWord*>(rt_address(p) & ~rt_pair_mask);
    const RtWord* const pairs = reinterpret_cast<const RtWord*>(rt_address(last) & ~rt_pair_mask);
    if (w != pairs) {
        RtWord pattern = v;
        pattern |= pattern << 8;
        pattern |= pattern << 16;
        RtWord lo = w[0] ^ pattern;
        RtWord hi = w[1] ^ pattern;
        rt_pair_head_masked(lo, hi, rt_address(p) & rt_pair_mask);
        RtWord first = rt_byte_marks(lo);
        RtWord both = rt_marks_over(first, hi);
        if (both == rt_no_zero_marks) {
            // The loop's one exit is the hit; the bound steps out by a
            // jump (the Zbb loop below says why).
            for (;;) {
                w += 2;
                if (w == pairs) {
                    p = reinterpret_cast<const unsigned char*>(w);
                    goto bytes;
                }
                lo = w[0] ^ pattern;
                hi = w[1] ^ pattern;
                first = rt_byte_marks(lo);
                both = rt_marks_over(first, hi);
                if (both != rt_no_zero_marks) {
                    break;
                }
            }
        }
        {
            // The pair that hit: the zero in its first word, or in its
            // second, whose marks the pair's are when the first has none.
            const unsigned char* q = reinterpret_cast<const unsigned char*>(w);
            if (first == rt_no_zero_marks) {
                q += rt_word_bytes;
                first = both;
            }
            return const_cast<unsigned char*>(q + rt_lowest_zero(first));
        }
    bytes:;
    }
#elif defined(RT_MARKS)
    const unsigned char* const last = p + n;
    const RtWord* w = reinterpret_cast<const RtWord*>(rt_address(p) & ~rt_word_mask);
    const RtWord* const words = reinterpret_cast<const RtWord*>(rt_address(last) & ~rt_word_mask);
    if (w != words) {
        RtWord pattern = v;
        pattern |= pattern << 8;
        pattern |= pattern << 16;
        RtWord m = rt_byte_marks(rt_head_masked(*w ^ pattern, rt_address(p) & rt_word_mask));
        if (m == rt_no_zero_marks) {
            // The loop's one exit is the hit; the bound steps out by a
            // jump. So shaped, GCC lays it out with one taken branch a
            // word; as a do-while with its two exits, the hit was the
            // fall-through and every turn took two, 9 cycles a word
            // against 7 (measured on Hazard3).
            rt_loop_on_word(w, pattern);
            for (;;) {
                if (++w == words) {
                    p = reinterpret_cast<const unsigned char*>(w);
                    goto bytes;
                }
                m = rt_byte_marks(*w ^ pattern);
                if (m != rt_no_zero_marks) {
                    break;
                }
            }
        }
        return const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(w) + rt_lowest_zero(m));
    bytes:;
    }
#else
    while ((rt_address(p) & rt_word_mask) != 0u) {
        if (n == 0u) {
            return nullptr;
        }
        if (*p == v) {
            return const_cast<unsigned char*>(p);
        }
        ++p;
        --n;
    }
    const unsigned char* const last = p + n;
    if (n >= rt_word_bytes) {
        RtWord pattern = v;
        pattern |= pattern << 8;
        pattern |= pattern << 16;
        // p is on a word boundary: its words end where last's word begins.
        const RtWord* w = reinterpret_cast<const RtWord*>(p);
        const RtWord* const words = reinterpret_cast<const RtWord*>(rt_address(last) & ~rt_word_mask);
        rt_loop_on_word(w, pattern);
        do {
            if (rt_has_zero(*w ^ pattern)) {
                break;
            }
            ++w;
        } while (w != words);
        p = reinterpret_cast<const unsigned char*>(w);
    }
#endif
    for (; p != last; ++p) {
        if (*p == v) {
            return const_cast<unsigned char*>(p);
        }
    }
    return nullptr;
}

}  // namespace

#ifndef BRIO_RT_HOST_TEST

extern "C" {

// On the M4 and the M33 memcpy is the leaf itself and takes the word
// boundary; elsewhere it is a frame around rt_copy_forward, which has it.
#if defined(__ARM_FEATURE_UNALIGNED)
[[gnu::aligned(4)]]
#endif
void* memcpy(void* __restrict dst, const void* __restrict src, size_t n) {
    return rt_memcpy(dst, src, n);
}

void* memmove(void* dst, const void* src, size_t n) {
    return rt_memmove(dst, src, n);
}

void* memset(void* dst, int c, size_t n) {
    return rt_memset(dst, c, n);
}

int memcmp(const void* a, const void* b, size_t n) {
    return rt_memcmp(a, b, n);
}

[[gnu::aligned(4)]] size_t strlen(const char* s) {
    return rt_strlen(s);
}

[[gnu::aligned(4)]] void* memchr(const void* s, int c, size_t n) {
    return rt_memchr(s, c, n);
}

// A spin, not a breakpoint: on ARMv6-M a breakpoint with no debugger
// attached escalates to HardFault and the frame that got here is lost,
// while a spin leaves the whole call stack for a debugger to walk. The
// empty asm is a side effect, so the loop is not one the language lets
// the compiler assume away. No panic breadcrumb: the runtime knows no
// platform (design/runtime.md).
[[noreturn]] void abort() {
    for (;;) {
        __asm__ volatile("");
    }
}

}  // extern "C"

#endif  // BRIO_RT_HOST_TEST

#pragma GCC pop_options
