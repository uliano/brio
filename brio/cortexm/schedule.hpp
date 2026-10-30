/*
 * schedule.hpp - the CORE stratum: the scheduling request of the
 * preemptive kernel, and the thread-mode scheduler it starts.
 *
 * What a Cortex-M platform gives Rubato (kernel/rubato.hpp) beyond what
 * Tenuto takes from it: a way to ask for the scheduler, and the place the
 * scheduler then runs. kernel/platform.hpp states the contract - the
 * request, pend_schedule(), runs the scheduler at once when it is made
 * from a dispatch and at the handler's exit when it is made from an
 * interrupt handler - and this file is how every Cortex-M keeps it. A
 * family's Rubato platform calls the request; a program binds the two
 * vectors this file needs to the bodies below, exactly as it binds every
 * other vector, and puts PendSV at its priority once:
 *
 *     using P = brio::Stm32f4RubatoPlatform<>;
 *     using K = brio::Rubato<P, Sensor, Logger>;
 *     extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
 *     extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }
 *     ...
 *     brio::scheduler_init();        // in main(), before interrupts are enabled
 *     K::run();
 *
 * Both handlers are NAKED: the bodies are assembly that moves the stack
 * pointer and returns from the exception itself, and a compiler-made
 * prologue would put a frame where they expect none. The bodies are
 * forced inline, so the vector's function is that assembly and nothing
 * else, in a release build and in a debug one.
 *
 * THE REQUEST IS PENDSV. PendSV is the exception the architecture keeps
 * for exactly this (DDI 0403E B1.5.1): pended by a write of
 * ICSR.PENDSVSET (B3.2.4), taken like an interrupt when its priority
 * allows. Its priority is set to the LOWEST level the core implements -
 * every bit of SHPR3's PRI_14 field (B3.2.12), 0xF0 on a four-bit M4,
 * 0xC0 on a two-bit M0+ - below every peripheral line, which stay at the
 * reset priority 0 (cortexm/nvic.hpp). So PendSV never interrupts a
 * peripheral handler: a request made from one waits, and the core takes
 * it at the handler's exit, by tail-chaining when nothing else is pending
 * (DDI 0439B 3.9.1: six cycles on the M4; on the RP2350's M33 seven
 * counted from the BX LR that ends the handler, where the M4's table
 * gives that BX and its tail-chain eight, measured by
 * test_rp2350_platform's letter i). From thread mode, with PRIMASK
 * clear, the request is taken before the instruction after the ISB that
 * follows the write: the DSB makes the write to the System Control Space
 * complete, the ISB makes the pending exception visible to the
 * instruction stream (A3.7.3, the synchronization requirements for
 * System Control Space updates), and that is what "at once" means for a
 * request made from a dispatch. A request made with PRIMASK set waits for
 * the unmask, so no dispatch ever starts inside a critical section; a
 * post made inside one serves its receiver when the section ends.
 *
 * WHERE THE SCHEDULER RUNS: IN THREAD MODE. PendSV does not run the
 * scheduler; it makes the core return from PendSV INTO it. With PRIMASK
 * set for the surgery, scheduler_exception<K>() pushes two words, then
 * builds a second exception frame below them, and returns through it:
 *
 *     higher addresses
 *       the real frame R         the core's, from the interrupted code:
 *                                8 words, or 26 with the FPU's (S0-S15,
 *                                FPSCR, a reserved word), maybe an
 *                                alignment word above
 *       EXC_RETURN of R          pushed by PendSV (with a filler below
 *       filler                   it, so the stack stays 8-aligned; the
 *                                entry writes `base` there, below)
 *       xPSR = 0x01000000        the fake frame F, 8 words: the Thumb
 *       PC   = the entry         bit alone in xPSR (IPSR 0, no alignment
 *       LR, R12, R3..R0          word), the entry's address as the return
 *                                address, the other six left as they are
 *     lower addresses            <- SP when PendSV returns through F
 *
 * The return value is R's own EXC_RETURN with bit 4 set: thread mode, the
 * main stack, a basic frame - 0xFFFFFFF9 on ARMv6-M and ARMv7-M,
 * 0xFFFFFFF9 too on an ARMv8-M running in Secure state, the security bits
 * kept from R's value wherever they are. PendSV is the only active
 * exception when it runs (it is the lowest; anything above it has
 * returned), so the return to thread mode passes the architecture's
 * check (B1.5.8). The core pops F and lands at scheduler_thread<K>() in
 * thread mode, SP just below the two pushed words, PRIMASK still set, and
 * PendSV no longer active.
 *
 * THE ENTRY AND THE EXIT (kernel/platform.hpp states what they owe the
 * kernel). Still masked, the entry reads the kernel's ceiling and stores
 * 0 in its place: from the request being taken to that store no handler
 * has run since PendSV's `cpsid`, so a handler that runs after it finds
 * the ceiling at 0, and its post asks for nothing and moves the count of
 * quiet posts. The entry then unmasks, writes the value it read - `base`
 * - into the filler word, and calls K::activate(base). activate() returns
 * with the ceiling still at 0 and the count its last choice read in its
 * result's low byte; the entry puts the count in r0 and the addresses of
 * the ceiling and of the count in r1 and r2, and executes `svc #0`.
 * scheduler_return(), the SVC handler, reads the three from the SVC's
 * frame - the stacked R0 to R2, which a handler that tail-chained ahead
 * of SVC cannot have changed, where the registers themselves it could -
 * and `base` from the filler under the frame, stores `base` back into the
 * ceiling and compares the count with the kernel's. When it moved, a
 * quiet post landed behind the last scan, and the handler pends PendSV
 * again: PENDSVSET, then a DSB, so that the write is complete when the
 * exception return looks for a pending exception (A3.7.3: an SCS write
 * takes effect when it completes, which a DSB guarantees). Either way the
 * handler then steps over the SVC's own frame (8 words: see the FPU
 * below), pops the filler and R's EXC_RETURN and returns with it. With
 * PendSV pending the return tail-chains into PendSV over R (the
 * RE-ENTRANCY paragraph), so the scheduler starts again in place of the
 * run that ended instead of over it; without, the core unstacks R, FPU
 * words included, and the interrupted code resumes with its registers as
 * the core saved them. R4-R11 and S16-S31 are never in a frame: they pass
 * through untouched, because everything between the two exceptions is
 * AAPCS code that preserves them.
 *
 * THE EXIT IS ONE STEP because SVCall runs at priority 0, the priority of
 * every peripheral line (cortexm/nvic.hpp): no peripheral handler
 * preempts it, PendSV is below it, and nothing runs between its store of
 * the ceiling and its exception return. A peripheral line pending at that
 * return is taken before PendSV, over R, at the ceiling just put back.
 *
 * WHAT THIS BUYS. While the dispatches run, PendSV is NOT active: an
 * interrupt handler's post pends it again, and it is taken at that
 * handler's exit, over whatever dispatch was running, at any depth - the
 * kernel's road 1. A dispatch's post to an object above it pends PendSV
 * from thread mode, taken at once, inside post() - road 2. Each level of
 * nesting costs, on the main stack, R's frame (32 or 104 bytes, 4 more
 * for alignment), the two pushed words and the scheduler's own frames;
 * the levels are the kernel's runs, at most one per object of the pack
 * (kernel/rubato.hpp, "The stack").
 *
 * RE-ENTRANCY. PendSV can be pended at any moment and the shapes all
 * close. Pended while a dispatch runs, it nests one level more, at that
 * dispatch's index. Pended between the entry's store and the `svc`, it
 * cannot be: the ceiling is 0 there, or a dispatch's index, and a post
 * that finds 0 asks for nothing. Pended while SVC_Handler runs - by
 * SVC_Handler itself, the exit's request - it is taken when SVC_Handler
 * returns, by tail-chaining: the core enters PendSV with R still on the
 * stack and R's EXC_RETURN in LR (B1.5.12's TailChain), exactly as if
 * PendSV had interrupted R directly, and builds its frame over R. And
 * pended by a peripheral handler that preempts PendSV before its first
 * instruction has run - the one moment between the request being taken
 * and the entry's store at which a handler can run - it is taken at the
 * entry's `cpsie`, after the store: that run finds the ceiling at 0,
 * serves nothing and returns, and the run it delayed serves the post
 * (kernel/rubato.hpp, "The stack", counts it). The two words and the fake
 * frame are written with PRIMASK set, and no peripheral handler touches
 * them.
 *
 * THE FPU (`#if __FPU_USED`, the M4F and the M33 with their FPU). The
 * lazy context save stays on (FPCCR.ASPEN and LSPEN at their reset value,
 * 1): an exception from code with an active FP context reserves R's FP
 * words and saves S0-S15 and FPSCR into them only when an FP instruction
 * runs before the return (B1.5.7). The fake frame is BASIC, so the return
 * through it makes CONTROL.FPCA 0 and leaves FPCCR.LSPACT as it was: the
 * first FP instruction a dispatch executes then saves the interrupted
 * code's registers into R, where they belong. Before its `svc` the entry
 * clears CONTROL.FPCA (the dispatches' FP context is over), so the SVC
 * frame is basic too and reserves nothing; R's EXC_RETURN, bit 4 clear
 * when R is an FP frame, then makes the final return restore S0-S15 and
 * FPSCR from R if they were saved, or leave them in the registers if
 * nothing touched them (B1.5.8's PopStack). A core without an FPU builds
 * none of this, and its frames are always basic.
 *
 * THE RULE IS NO BARRIER IN THE BODY, AND THE CORTEX-M4 IS THE EXCEPTION.
 * The exception return itself orders the bodies' stores on every core of
 * the architecture; only the Cortex-M4 r0p1 (the STM32F4's) has erratum
 * 838869, a store just before an exception return letting the core
 * vector to the wrong interrupt, and ARM's workaround is a DSB after the
 * last store. So PendSV's body and SVC's carry that DSB on a Cortex-M4
 * alone (the device header's __CORTEX_M), three cycles the Cortex-M0+ and
 * the Cortex-M33 do not pay for an erratum they have not got. The DSB
 * after the exit's PENDSVSET is another matter, the architecture's rule
 * for an SCS write whose effect must be in place before what follows, and
 * every core pays it, on that path alone.
 *
 * WHAT THIS FILE ASSUMES. Thread mode runs on the main stack
 * (CONTROL.SPSEL 0, the reset value, which no brio crt changes); every
 * peripheral line stays at priority 0, as cortexm/nvic.hpp says, so that
 * PendSV is below them all, and so does SVCall (SHPR2's reset value, which
 * nothing in brio writes), so that no peripheral handler runs inside the
 * exit; nothing but this file writes PendSV's priority or pends it; and
 * the program executes no `svc` of its own - SVC_Handler is this file's.
 *
 * WHICH CORES. ARMv6-M (the M0+ of the SAM C21, the STM32G0, the RP2040),
 * ARMv7-M (the M4 of the STM32F4) and ARMv8-M main (the M33 half of the
 * RP2350): PendSV, SVCall, ICSR.PENDSVSET, SHPR3, the exception frame and
 * EXC_RETURN's low five bits are the same programmer's model on the
 * three (SHPR3 is word-access only on ARMv6-M, which CMSIS's
 * NVIC_SetPriority honours). The assembly is the ARMv6-M subset - no IT
 * block, no load into the PC, the 16-bit forms only - so the one source
 * assembles for all three. Five families' platforms take it today and
 * have run it: the STM32F4's (stm32f4/platform.hpp's
 * Stm32f4RubatoPlatform, the M4), the STM32G0's, the SAM C21's and the
 * RP2040's on the M0+ (stm32g0/platform.hpp, samc21/platform.hpp,
 * rp2040/platform.hpp), and the RP2350's on its Cortex-M33
 * (rp2350/platform.hpp's Rp2350RubatoPlatform).
 *
 * Include-order contract: the family's device header first (it brings
 * the CMSIS core header with SCB, PendSV_IRQn and __FPU_USED); the
 * family's nvic.hpp does, and this file includes cortexm/nvic.hpp after
 * it.
 */

#pragma once

#include <stdint.h>

#if !defined(__CM0PLUS_REV) && !defined(__CM0_REV) && !defined(__CM4_REV) && !defined(__CM33_REV)
#error "cortexm/schedule.hpp: include the family's device header first (stm32f4/platform.hpp does) - the CMSIS core header it brings is what this file is written against"
#endif

#include <stddef.h>

#include "cortexm/nvic.hpp"
#include "kernel/platform.hpp"

namespace brio {

/// The request: PENDSVSET, with the DSB and the ISB that make it taken
/// before the next instruction in thread mode, at the handler's exit in a
/// handler.
///
/// A CALL THE COMPILER CANNOT SEE INTO, on purpose: noinline and noipa.
/// From a dispatch the request runs other dispatches before it returns,
/// through an exception the compiler does not know of, so to the caller
/// it must be a call to code that may read and write any static of the
/// program - which an out-of-line function with no interprocedural
/// summary is. The DSB's "memory" clobber is not enough: GCC 16 at -Os,
/// knowing what the request's body touches, deleted as dead a store to a
/// static the receiver read, because the caller stored it again after
/// the post (test_stm32f4_rubato's letter f caught it: the receiver ran
/// with the value of before). The price is one call and return on the asking path only; a
/// post that asks nothing never reaches it.
[[gnu::noinline, gnu::noipa]] inline void request_scheduler() {
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;   // every other ICSR bit written 0: no effect (B3.2.4)
    __DSB();
    __ISB();
}

/// PendSV to the lowest priority the core implements, every bit of the
/// field. Call once in main(), before interrupts are enabled.
inline void scheduler_init() {
    NVIC_SetPriority(PendSV_IRQn, irq_priority_levels - 1u);
}

/// The thread-mode entry the fake frame returns into (the file header):
/// still masked, the ceiling read and 0 stored in its place; then the
/// unmask, the value read put in the filler word, the scheduler called
/// with it, the FP context cleared (where there is an FPU), and the way
/// out through SVC with the count of quiet posts and the two addresses
/// the exit needs in r0 to r2. Naked: it owns no frame, so the SP at its
/// `svc` is the SP it started with, and it touches no register the
/// interrupted code keeps outside the frame. Twelve instructions, sixteen
/// with the FPU's four, four of them masked.
template <class Kernel>
[[gnu::naked, gnu::noinline]] void scheduler_thread() {
    asm volatile(
        ".syntax unified\n\t"
        "ldr r1, =%c1\n\t"            // the kernel's ceiling
        "ldrb r0, [r1]\n\t"           // base: the ceiling the request was taken at
        "movs r2, #0\n\t"
        "strb r2, [r1]\n\t"           // the ceiling 0, before any handler can run
        "cpsie i\n\t"
        "str r0, [sp]\n\t"            // the filler: base, for the exit
        "bl %c0\n\t"                  // activate(base): the count in r0's low byte
        "uxtb r0, r0\n\t"
#if defined(__FPU_USED) && (__FPU_USED == 1U)
        "mrs r3, control\n\t"
        "bic r3, r3, #4\n\t"          // CONTROL.FPCA: the SVC frame is basic
        "msr control, r3\n\t"
        "isb\n\t"
#endif
        "ldr r1, =%c1\n\t"            // the SVC frame's R1: the ceiling
        "ldr r2, =%c2\n\t"            // its R2: the count of quiet posts
        "svc #0\n\t"
        "b .\n\t"                     // never reached: SVC returns into the real frame
        ".ltorg\n\t"
        :: "i"(&Kernel::activate), "i"(&Kernel::Platform::ceiling),
           "i"(&quiet_posts<typename Kernel::Platform>));
}

#if (__CORTEX_M == 4U)
/// The DSB erratum 838869 wants before an exception return that follows
/// a store: the Cortex-M4's alone (the file header).
#define BRIO_CORTEXM_EXCEPTION_RETURN_DSB "dsb\n\t"
#else
#define BRIO_CORTEXM_EXCEPTION_RETURN_DSB
#endif

/// The body the program binds to PendSV_Handler, a naked handler: the two
/// words and the fake frame, then the return into scheduler_thread<K>().
/// Thirteen instructions; fourteen on the Cortex-M4, whose erratum
/// 838869 wants a DSB before the return (the file header).
template <class Kernel>
[[gnu::always_inline]] inline void scheduler_exception() {
    asm volatile(
        ".syntax unified\n\t"
        "cpsid i\n\t"
        "push {r0, lr}\n\t"         // a filler, then R's EXC_RETURN
        "sub sp, sp, #32\n\t"       // the fake frame
        "ldr r0, =%c0\n\t"          // the entry, its Thumb bit set
        "subs r0, r0, #1\n\t"       // the stacked return address is halfword-aligned
        "str r0, [sp, #24]\n\t"     // F.PC
        "movs r0, #1\n\t"
        "lsls r0, r0, #24\n\t"
        "str r0, [sp, #28]\n\t"     // F.xPSR: the Thumb bit alone
        "mov r0, lr\n\t"
        "movs r1, #16\n\t"
        "orrs r0, r0, r1\n\t"       // R's EXC_RETURN, a basic frame
        BRIO_CORTEXM_EXCEPTION_RETURN_DSB   // the Cortex-M4 alone: erratum 838869
        "bx r0\n\t"
        ".ltorg\n\t"
        :: "i"(&scheduler_thread<Kernel>));
}

/// The body the program binds to SVC_Handler, a naked handler: the exit
/// (the file header). From the SVC's frame the count of quiet posts the
/// scheduler read and the addresses of the ceiling and of the count, from
/// the filler under it the ceiling to put back; the ceiling stored, the
/// count compared, PendSV pended again when it moved; then the SVC's frame
/// stepped over and the return into R with R's EXC_RETURN. Eleven
/// instructions on the path that finds the count still, twelve on the
/// Cortex-M4 (its erratum's DSB); five more, a branch out and back
/// included, on the path that pends. Not a template: the addresses come
/// with the frame, so one body serves every kernel, the two of a pair of
/// cores included.
[[gnu::always_inline]] inline void scheduler_return() {
    asm volatile(
        ".syntax unified\n\t"
        "ldr r0, [sp, #0]\n\t"        // the SVC frame's R0: the count the scheduler read
        "ldr r1, [sp, #4]\n\t"        // R1: the ceiling
        "ldr r2, [sp, #8]\n\t"        // R2: the count of quiet posts
        "ldr r3, [sp, #32]\n\t"       // the filler: base
        "strb r3, [r1]\n\t"           // the ceiling back
        "ldrb r2, [r2]\n\t"
        "cmp r0, r2\n\t"
        "bne 2f\n"
        "1:\n\t"
        "add sp, sp, #32\n\t"         // the SVC's own frame: 8 words, never padded
        "pop {r0, r1}\n\t"            // the filler, R's EXC_RETURN
        BRIO_CORTEXM_EXCEPTION_RETURN_DSB   // the Cortex-M4 alone: erratum 838869
        "bx r1\n"
        "2:\n\t"
        "ldr r0, =%c0\n\t"            // a quiet post landed behind the last scan:
        "ldr r1, =%c1\n\t"
        "str r1, [r0]\n\t"            // PendSV pended again, taken by tail-chaining
        "dsb\n\t"                     // the write complete before the return (A3.7.3)
        "b 1b\n\t"
        ".ltorg\n\t"
        :: "i"(SCB_BASE + offsetof(SCB_Type, ICSR)), "i"(SCB_ICSR_PENDSVSET_Msk));
}

} // namespace brio
