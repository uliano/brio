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
 * left by 32 - 8k, k the source's offset in its word, 1 to 3). There no
 * function here performs a word access at an address that is not a
 * multiple of the word.
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
 * replicated across the word, then the bytes of the word that hit. A
 * RISC-V core with the Zbb extension (__riscv_zbb: Hazard3 has it, the
 * QingKe cores have not) tests the word with orc.b instead, which writes
 * every byte of its result as 0xFF where the byte read is not zero and
 * 0x00 where it is: the zero bytes marked exactly in one instruction,
 * the lowest one's position the count of the marks' trailing ones
 * divided by eight (ctz of the complement), no byte loop after the hit.
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
 * alike. A RISC-V core has no such instruction; there a block is four
 * loads and four stores.
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

// The fill from its first word boundary to its end: whole blocks, then
// words, then bytes - the byte being the word's low one. On a Thumb core
// what is left after the turns; on another, all of it.
[[gnu::always_inline]] inline void rt_fill_rest(unsigned char* d, const RtWord w, size_t n) {
    const size_t span = n & ~(rt_block_bytes - 1u);
    if (span != 0u) {
        unsigned char* const blocks = d + span;
        do {
            reinterpret_cast<RtWord*>(d)[0] = w;
            reinterpret_cast<RtWord*>(d)[1] = w;
            reinterpret_cast<RtWord*>(d)[2] = w;
            reinterpret_cast<RtWord*>(d)[3] = w;
            d += rt_block_bytes;
        } while (d != blocks);
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

// The whole blocks of n from s to d, both advanced: four loads and four
// stores a turn.
[[gnu::always_inline]] inline void rt_copy_blocks(unsigned char*& d, const unsigned char*& s, const size_t n) {
    const size_t span = n & ~(rt_block_bytes - 1u);
    if (span != 0u) {
        unsigned char* const blocks = d + span;
        do {
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
        } while (d != blocks);
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
// lo >> 8K | hi << (32 - 8K), K the source's offset in its word. The
// shifts are constants, one instance per offset: with the shifts in
// registers the loop spills to the stack on the Cortex-M0+ (whose Thumb-1
// instructions reach eight registers) and on the QingKe V2 (RV32E, sixteen),
// and a constant shift is no instruction more than a variable one. Thumb-1
// runs a word a turn (four a turn spill again, measured slower on the
// STM32G0), and so does RV32E, the QingKe V2 of the CH32V00x parts: there
// four a turn cost some 300 bytes, which the 15 KB image of the CH32V003
// does not have to spare, for 10 to 16 per cent at 60 bytes on the
// CH32V006. The other RISC-V cores run four a turn, then single words.
template <unsigned K>
[[gnu::always_inline]] inline void rt_merge_words(unsigned char*& d, const RtWord* from, RtWord lo, size_t n) {
    constexpr unsigned right = 8u * K;
    constexpr unsigned left = 32u - right;
#if !defined(__thumb__) && !defined(__riscv_e)
    const size_t span = n & ~(rt_block_bytes - 1u);
    if (span != 0u) {
        unsigned char* const blocks = d + span;
        do {
            const RtWord w0 = from[0];
            const RtWord w1 = from[1];
            const RtWord w2 = from[2];
            const RtWord w3 = from[3];
            reinterpret_cast<RtWord*>(d)[0] = (lo >> right) | (w0 << left);
            reinterpret_cast<RtWord*>(d)[1] = (w0 >> right) | (w1 << left);
            reinterpret_cast<RtWord*>(d)[2] = (w1 >> right) | (w2 << left);
            reinterpret_cast<RtWord*>(d)[3] = (w2 >> right) | (w3 << left);
            lo = w3;
            from += 4;
            d += rt_block_bytes;
        } while (d != blocks);
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
// words move (the file's header: why that is sound).
[[gnu::always_inline]] inline void rt_copy_words_across(unsigned char*& d, const unsigned char*& s, size_t n) {
    const uintptr_t k = rt_address(s) & rt_word_mask;
    const RtWord* const from = reinterpret_cast<const RtWord*>(s - k);
    const RtWord lo = from[0];
    s += n & ~rt_word_mask;
    if (k == 1u) {
        rt_merge_words<1>(d, from + 1, lo, n);
    } else if (k == 2u) {
        rt_merge_words<2>(d, from + 1, lo, n);
    } else {
        rt_merge_words<3>(d, from + 1, lo, n);
    }
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
// and memchr: where the C extension packs code at halfwords, a 32-bit
// instruction across a word costs a fetch, and the QingKe V4B's co-aligned
// copy of 4096 bytes measured 3412 cycles with its loop at an odd
// halfword against 3156 on a word. OUT OF LINE, memcpy a frame around its
// call (two pushes, a call and a return: some 13 cycles on the Cortex-M0+
// and 6 to 8 on the RISC-V cores, counted in the listings), because the
// forms that spend less were read and measured worse: inlined into memcpy,
// which must keep dst for its return, GCC on the M0+ saved eight registers
// and kept the count on the stack (within a few cycles of this form either
// way in letter m), and the QingKe V4B's block loop moved to an odd
// halfword, 3156 to 3410 cycles at 4096 bytes aligned alike; handed dst to
// return itself, so that memcpy is a tail call, the M0+ spilled it and
// made no tail call (GCC makes none on Thumb-1), and the V4B's loop moved
// again.
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

#if defined(__riscv_zbb)
// THE SCANS WITH THE ZBB EXTENSION (the file's header says what orc.b
// does). The marks of a word: 0xFF for every byte that is not zero, 0x00
// for every zero byte; a word with no zero byte marks as all ones.
[[gnu::always_inline]] inline RtWord rt_byte_marks(const RtWord w) {
    return __riscv_orc_b_32(w);
}

constexpr RtWord rt_no_zero_marks = ~RtWord{0};

// The index of the lowest zero byte of a word that has one, from its
// marks: the complement's trailing zeros are eight per byte below it.
[[gnu::always_inline]] inline size_t rt_lowest_zero(const RtWord marks) {
    return static_cast<size_t>(__riscv_ctz_32(~marks)) >> 3;
}
#endif

// The length of a zero-terminated string: bytes up to a word boundary,
// then aligned words until one holds a zero byte, then its bytes - or,
// under Zbb, the zero byte's index in the word that hit.
size_t rt_strlen(const char* s) {
    const char* p = s;
    while ((rt_address(p) & rt_word_mask) != 0u) {
        if (*p == '\0') {
            return static_cast<size_t>(p - s);
        }
        ++p;
    }
    const RtWord* w = reinterpret_cast<const RtWord*>(p);
#if defined(__riscv_zbb)
    RtWord m;
    do {
        m = rt_byte_marks(*w++);
    } while (m == rt_no_zero_marks);
    // Written from w, not from w - 1, for the loop GCC makes of it: four
    // instructions a word (the load, the step, orc.b, the compare), where
    // keeping the hit word's address costs a fifth.
    return static_cast<size_t>(reinterpret_cast<const char*>(w) - s) - rt_word_bytes + rt_lowest_zero(m);
#else
    RtWord x;
    do {
        x = *w++;
    } while (!rt_has_zero(x));
    p = reinterpret_cast<const char*>(w - 1);
    while (*p != '\0') {
        ++p;
    }
#endif
    return static_cast<size_t>(p - s);
}

// The first byte equal to (unsigned char)c within n bytes, or nothing:
// bytes up to a word boundary, then aligned words, each xored with the
// byte replicated, until one holds a zero byte, then the bytes from it to
// the bound - under Zbb, the zero byte's index in the word that hit,
// which lies within the bound because the words end where the bound's
// own word begins.
void* rt_memchr(const void* s, int c, size_t n) {
    const auto* p = static_cast<const unsigned char*>(s);
    const auto v = static_cast<unsigned char>(c);
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
#if defined(__riscv_zbb)
        // The loop's one exit is the hit; the bound steps out by a jump.
        // So shaped, GCC lays it out as the load, the exclusive or,
        // orc.b, a compare not taken, the step and the branch back: six
        // instructions and one taken branch a word. As a do-while with
        // its two exits, the hit was the fall-through and every turn took
        // two taken branches, 9 cycles a word against 7 (measured).
        RtWord m;
        for (;;) {
            m = rt_byte_marks(*w ^ pattern);
            if (m != rt_no_zero_marks) {
                break;
            }
            if (++w == words) {
                goto bytes;
            }
        }
        return const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(w) + rt_lowest_zero(m));
    bytes:
#else
        do {
            if (rt_has_zero(*w ^ pattern)) {
                break;
            }
            ++w;
        } while (w != words);
#endif
        p = reinterpret_cast<const unsigned char*>(w);
    }
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
