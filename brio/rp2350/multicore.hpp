/*
 * multicore.hpp
 *
 * The second core of the RP2350 as brio sees it (datasheet 3.1.2 the
 * SIO's CPUID, 3.1.5 the inter-processor FIFOs, 3.1.6 the doorbells,
 * 3.3 and 3.4 the two architectures' event signals, 5.2 and 5.3 the
 * bootrom's wait-for-launch and its protocol, 7.4 the power-on state
 * machine that holds one core in reset): the doorbell util/inbox.hpp
 * rings, and the launch of core 1 - on EITHER architecture, from one
 * source.
 *
 * THE MODEL IS TWO KERNELS, NOT ONE ON TWO CORES (util/inbox.hpp's
 * header, design/kernel.md section 12): every AO lives on one core, in
 * the pack of that core's kernel, on that core's platform type
 * (Rp2350Platform<0> or <1>); events cross through inboxes; a peripheral
 * and its interrupt line belong to the core whose kernel hosts the
 * driver - every system line reaches BOTH interrupt controllers, so the
 * owning core is the one that enables it, and a line enabled on both
 * runs its handler twice.
 *
 * THE DOORBELL IS THE DOORBELL REGISTERS, AND NOT THE MAILBOX FIFO -
 * the one place this file departs from its RP2040 ancestor, and the
 * reason is 3.1.6's own sentence: the FIFOs are for "cross-core events
 * whose count and order is important", the doorbells for events "which
 * are accumulative (i.e. may post multiple times, but only answered
 * once) and which can be responded to in any order". That is the bell
 * exactly: the bridge's counting and ordering live in the inbox, and
 * what the bell carries is "look at your inboxes". Three things follow,
 * and all three are gains over a FIFO bell:
 *
 *  - A RING NEVER FAILS AND NEVER BLOCKS. Eight flags stand in each
 *    direction; raising one that already stands is idempotent, where a
 *    FIFO write into a full FIFO is a lost bell and a sticky error flag.
 *  - THE BELL AND THE LAUNCH DO NOT SHARE A CHANNEL. The launch protocol
 *    below is the FIFO's alone and the bell is SIO_IRQ_BELL, a different
 *    line: core 0's bell interrupt may stay enabled across a launch, and
 *    a send that lands in the middle of one disturbs nothing. On the
 *    RP2040 the two shared the FIFO and the launch had to run with the
 *    doorbell interrupt off.
 *  - A RING IS CORRECT FROM EITHER CORE. DOORBELL_OUT_SET raises the
 *    OPPOSITE core's bell and DOORBELL_IN_SET this core's own, so
 *    `SioDoorbell<c>::ring()` picks by CPUID and means "ring core c"
 *    wherever it is called - 3.1.6 names that case ("useful when the
 *    routine which rings a doorbell can be scheduled on either core").
 *
 * The bridge owns ALL EIGHT flags of the core it rings: it raises flag 0
 * and `pop_all()` acknowledges whatever stands, the way the RP2040's
 * bell drained the whole FIFO. A program that wants a second cross-core
 * channel of its own takes the mailbox FIFO, which nothing but the
 * launch uses here.
 *
 * ERRATUM RP2350-E2 AND THIS FILE. On every stepping to date a WRITE to
 * an SIO register at offset 0x180 or above is also decoded as a write to
 * the spinlock 128 bytes below it, releasing that lock: the four
 * doorbell registers alias SPINLOCK0..3. brio takes no SIO spinlock on
 * this chip - the kernel's exclusion is per core and the bridge needs
 * none - so what the erratum costs this bridge is nothing; a program
 * that does take one may not take locks 0 to 3 (nor those the platform
 * timer's registers alias, rp2350/mtime.hpp), and the erratum's own text
 * lists the ones that stay safe.
 *
 * THE LAUNCH is the bootrom's protocol (5.3): after a reset core 1 waits
 * in the ROM, draining its FIFO and echoing what it reads, and core 0
 * sends 0, 0, 1, a vector base, a stack pointer and an entry point, each
 * word answered by its echo, the sequence restarted from the top on any
 * wrong answer, a 0 preceded by a drain of core 0's own inbound FIFO and
 * by an EVENT (`sev` on the Cortex-M33 half, `h3.unblock` on the Hazard3
 * one) because core 1 may be asleep waiting for FIFO room. The same six
 * words on both architectures: only what the fourth one means differs -
 * VTOR there, mtvec here - and the entry point carries the Thumb bit on
 * the Arm half for free, a C++ function pointer having it already.
 *
 * THE LAUNCH RESETS CORE 1 FIRST. Only a core 1 in the ROM's wait loop
 * answers the protocol, and core 1 is not there whenever core 0 alone
 * was reset - which is what a probe's `reset run` and `Reset::core()`
 * do, leaving core 1 running its old life. `reset()` puts it back
 * through the power-on state machine's FRCE_OFF.PROC1 (7.4), the one
 * stage of that machine a program may hold without stopping itself, and
 * pops the 0 core 1 pushes once it has drained its FIFO; `launch()` then
 * bounds every wait and answers false when the protocol does not
 * complete (`protocol()` alone is for a core 1 known to be waiting).
 * Erratum RP2350-E19 - a reboot hangs with any FRCE_OFF bit but PROC1
 * set - is not this file's to answer twice: PROC1 is the only bit it
 * ever sets, and the watchdog's reboot clears the others before it
 * triggers (rp2350/watchdog.hpp).
 *
 * WHAT THE BOOTROM DOES NOT HAND OVER, and what this file's own entry
 * shim does on core 1 before the program's entry runs. The ROM sets the
 * stack pointer and the vector base and jumps (table 451); everything
 * else of a core's private state is the program's:
 *
 *  - on the Hazard3 half, THE GLOBAL POINTER. gp is a register, not a
 *    memory location, and the crt sets it once - on core 0. Core 1 must
 *    set its own before any code that the linker relaxed into a
 *    gp-relative access runs, which is every small global in the image.
 *  - on the Cortex-M33 half, MSPLIM and CPACR. The stack limit is
 *    per core and whatever the ROM left in it would fault core 1's
 *    first push; CP10 and CP11 are per core too, and this project
 *    builds the M33 with the HARD-FLOAT ABI, so a core 1 that touches
 *    a float without them takes a UsageFault (the crt does the same
 *    two acts for core 0, and for the same two reasons).
 *
 * The shim is therefore in two halves. `core1_trampoline()` is NAKED and
 * carries NO RELOCATION AT ALL - it reads four words off the stack core
 * 0 prepared for it, because a symbol reference is exactly what cannot
 * be trusted before gp exists - and jumps to `Core1::start()`, an
 * ordinary function with the frame's entry point and vector base in its
 * two argument registers. `start()` adopts the vector base, does the
 * per-core acts above, marks that core 1 arrived, and calls the
 * program's entry.
 *
 * What core 1's entry does is the program's: its own ticker
 * (`CoreTicker<1>`: its own SysTick on the Arm half, its own comparator
 * against the shared microsecond counter on the RISC-V one), the lines
 * its drivers own enabled in ITS controller, its bell enabled, interrupts
 * on, its kernel's run(). What it must not do: touch the clock tree
 * (core 0 owns it), or post to an AO of core 0 (the queue refuses and
 * counts).
 *
 * THE PREPROCESSOR IS ASKED HERE, about BRIO_RP2350_CORE_M33 and never
 * about the compiler's `__riscv` - the same question rp2350/ticker.hpp
 * and rp2350/reset.hpp ask, for the same kind of reason: an Arm register
 * name does not exist in the RISC-V build at all, so `if constexpr` is
 * not enough - the name would still be looked up. Where both spellings
 * do compile everywhere (an instruction inside an asm string), the
 * choice is a template's `if constexpr` on `core_kind` and no macro.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "rp2350/core.hpp"
#include "rp2350/resets.hpp"

/// The linker script's symbol for the top of core 1's stack: the top of
/// the unstriped bank scratch_x, reserved and untouched until a second
/// kernel runs there (rp2350/ld/).
extern "C" char __stack_one_top;

namespace brio {

/// POST AN EVENT TO THE OTHER CORE - not an interrupt, the sleep-state
/// nudge the launch protocol needs because core 1 waits in the ROM with
/// the architecture's own wait instruction. `sev` on the Cortex-M33 half
/// (3.3), `h3.unblock` on the Hazard3 one (3.4, 3.8.6.3.2: `slt x0, x0,
/// x1`, which is a NOP anywhere else). Both are sticky: an event posted
/// before the other core sleeps makes its sleep fall through.
///
/// It lives here rather than beside core.hpp's verbs because the launch
/// protocol is its only caller; the kernel's idle path sleeps on
/// interrupts, not on events.
template <CoreKind k = core_kind>
[[gnu::always_inline]] inline void core_send_event() {
    if constexpr (k == CoreKind::hazard3) {
        __asm__ volatile("slt x0, x0, x1" ::: "memory");
    } else {
        __asm__ volatile("sev" ::: "memory");
    }
}

/**
 * The bell towards `core`: that core's inbound doorbell flags, and the
 * line they raise on it.
 *
 * SIO_IRQ_BELL is CORE-LOCAL and has the SAME NUMBER on both cores
 * (3.1.6): enabling it enables the bell of the core that runs the verb,
 * which is why `enable()` refuses on any other - on the RP2040, where
 * the two cores' bells were two different line numbers, the same mistake
 * was merely useless.
 */
template <uint8_t core>
struct SioDoorbell {
    static_assert(core < 2u, "the RP2350 has two cores per architecture, 0 and 1");
    SioDoorbell() = delete;

    /// Every flag of one direction: eight, and the bridge owns them all
    /// (the file header).
    static constexpr uint32_t flags = SIO_DOORBELL_OUT_SET_BITS;
    /// The one a ring raises. Which it is does not matter to a bell;
    /// that it is always the same one makes a bench letter able to name
    /// what it sees.
    static constexpr uint32_t bell = 1u;

    /// The line, the same number whichever core takes it.
    static constexpr IRQn_Type irq() { return SIO_IRQ_BELL_IRQn; }

    /// RING CORE `core`, from either core: the opposite core's flags
    /// through DOORBELL_OUT_SET, this core's own through DOORBELL_IN_SET
    /// (3.1.6). Idempotent - a flag that already stands is already a
    /// bell - and followed by the architecture's event, so a core
    /// sleeping in a wait instruction wakes even before its handler
    /// runs.
    static void ring() {
        if (SIO->CPUID == core) {
            SIO->DOORBELL_IN_SET = bell;
        } else {
            SIO->DOORBELL_OUT_SET = bell;
        }
        core_send_event();
    }

    /// ON THIS CORE: acknowledge every flag that stands, and return what
    /// stood. The interrupt deasserts once they are all clear, which is
    /// why this runs FIRST in the drain (util/inbox.hpp's order).
    ///
    /// There is no verb for acknowledging the OTHER core's bells,
    /// although DOORBELL_OUT_CLR would: 3.1.6 warns that it races the
    /// core whose interrupt it is, and nothing here needs it.
    static uint32_t pop_all() {
        const uint32_t raised = SIO->DOORBELL_IN_CLR & flags;
        if (raised != 0u) {
            SIO->DOORBELL_IN_CLR = raised;
        }
        return raised;
    }

    /// Whether a bell stands for `core`, asked from either core: this
    /// core's inbound flags, or the opposite core's through the
    /// read-back of the outbound register (3.1.6: both spellings of each
    /// pair read the same status).
    static bool pending() {
        const uint32_t standing =
            (SIO->CPUID == core) ? SIO->DOORBELL_IN_CLR : SIO->DOORBELL_OUT_SET;
        return (standing & flags) != 0u;
    }

    /// The bell interrupt on, in the controller of the core that runs
    /// this - so it must BE `core`. False and nothing enabled otherwise:
    /// one line number serves both cores here, so enabling it elsewhere
    /// would arm the wrong core's bell.
    static bool enable() {
        if (SIO->CPUID != core) {
            return false;
        }
        Irq::enable(irq());
        return true;
    }

    static bool disable() {
        if (SIO->CPUID != core) {
            return false;
        }
        Irq::disable(irq());
        return true;
    }
};

/// The four words `Core1::launch()` leaves on core 1's stack for the
/// trampoline to pop. NO SYMBOL REACHES THE TRAMPOLINE: every address it
/// needs is here, written by core 0, which has a global pointer and a
/// linker.
struct Core1Frame {
    uint32_t entry;         ///< the program's entry, into the first argument register
    uint32_t global_ptr;    ///< gp on the Hazard3 half; unused on the Arm one
    uint32_t start;         ///< Core1::start, jumped to
    uint32_t vectors;       ///< the vector base, into the second argument register
};

/// THE FIRST INSTRUCTIONS CORE 1 RUNS. Naked, relocation-free (the file
/// header says why), and the ABI is what makes it short: the frame's
/// words land in the argument registers `start()` declares.
[[gnu::naked]] inline void core1_trampoline() {
#if defined(BRIO_RP2350_CORE_M33)
    // r0 = entry, r1 = vectors, r2 = start; MSPLIM cleared before
    // anything can push, since what the ROM left there is not ours.
    __asm__ volatile("ldr r0, [sp, #0]\n"
                     "ldr r1, [sp, #12]\n"
                     "ldr r2, [sp, #8]\n"
                     "add sp, sp, #16\n"
                     "movs r3, #0\n"
                     "msr msplim, r3\n"
                     "bx r2\n");
#else
    // a0 = entry, a1 = vectors, t0 = start, and gp before any of them is
    // used for anything.
    __asm__ volatile("lw a0, 0(sp)\n"
                     "lw a1, 12(sp)\n"
                     "lw gp, 4(sp)\n"
                     "lw t0, 8(sp)\n"
                     "addi sp, sp, 16\n"
                     "jr t0\n");
#endif
}

/// This core's global pointer, for the frame: the value core 0 runs on,
/// which is the image's one anchor and so core 1's too.
template <CoreKind k = core_kind>
inline uint32_t core_global_pointer() {
    if constexpr (k == CoreKind::hazard3) {
        uint32_t value;
        __asm__ volatile("mv %0, gp" : "=r"(value));
        return value;
    } else {
        return 0u;
    }
}

/// Where this core's exception vectors are: VTOR on the Cortex-M33 half,
/// mtvec - mode bit and all - on the Hazard3 one. What the launch hands
/// core 1, and what `start()` writes there once it is running.
inline uint32_t core_vector_base() {
#if defined(BRIO_RP2350_CORE_M33)
    return SCB->VTOR;
#else
    return csr_read<RVCSR_MTVEC_OFFSET>();
#endif
}

/**
 * Core 1: launched by core 0, put back into the ROM by core 0.
 *
 * A TEMPLATE WITH NOTHING TO CHOOSE, so that its statics and its entry
 * shim exist only in a program that launches a second core: an alias to
 * a class template instantiates no member until one is used, and
 * `rp2350/platform.hpp` includes this file for every program on this
 * target.
 */
template <typename = void>
struct Core1Of {
    Core1Of() = delete;

    /// The stack the linker script reserves for core 1: the top of the
    /// unstriped bank scratch_x, so core 1's stack traffic contends with
    /// neither core 0's nor the DMA's on a striped bank.
    static uint32_t default_stack_top() {
        return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&__stack_one_top));
    }

    /// The words core 1 answered in the last launch, for a bench suite's
    /// eyes (at most the last sixteen).
    static inline uint32_t trace[16]{};
    static inline uint32_t trace_count = 0;

    /// Whether core 1 has reached `start()` since the last attempt - the
    /// entry shim's own word, and the proof that the protocol's last
    /// echo was followed by an arrival.
    static bool entered() { return entered_; }

    /// Core 1 reset into the ROM, then the protocol: `entry` runs on core
    /// 1 with `stack_top` as its stack and this core's vector base. False
    /// when the reset went unanswered or the sequence did not complete
    /// within its bounded waits.
    static bool launch(void (*entry)(), uint32_t stack_top = default_stack_top()) {
        return reset() && protocol(entry, stack_top);
    }

    /// The bootrom's launch protocol alone (the file header), for a core
    /// 1 known to be in the ROM's wait loop: false when it did not answer
    /// the sequence within the bounded waits - it is not there (running:
    /// reset() first), the FIFO is wedged, or this core's own FIFO
    /// interrupt is enabled, whose handler would eat the echoes. The
    /// BELL interrupt may stay on: it is another line and another
    /// register (the file header).
    ///
    /// THE SIXTEEN BYTES BELOW `stack_top` ARE WRITTEN BEFORE THE FIRST
    /// WORD GOES OUT - the frame the trampoline pops - so `stack_top`
    /// must not be the top of a stack something is standing on. After a
    /// reset() nothing is, which is what makes launch() safe; a caller
    /// PROBING a running core 1 hands a scratch area of its own instead,
    /// and gets its refusal without touching core 1's stack.
    static bool protocol(void (*entry)(), uint32_t stack_top = default_stack_top()) {
        if (Irq::enabled(SIO_IRQ_FIFO_IRQn)) {
            return false;
        }
        const uint32_t sp = (stack_top - sizeof(Core1Frame)) & ~15u;
        Core1Frame& frame = *reinterpret_cast<Core1Frame*>(static_cast<uintptr_t>(sp));
        frame.entry = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(entry));
        frame.global_ptr = core_global_pointer();
        frame.start = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&start));
        frame.vectors = core_vector_base();
        entered_ = false;

        // The Thumb bit of 5.3's note is already set: the address of a
        // function is what this architecture calls through.
        const uint32_t sequence[] = {
            0u, 0u, 1u, frame.vectors, sp,
            static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&core1_trampoline))};
        constexpr uint32_t words = sizeof sequence / sizeof sequence[0];
        uint32_t seq = 0;
        uint32_t attempts = 64;   // restarts of the sequence, bounded
        trace_count = 0;
        while (seq < words) {
            const uint32_t cmd = sequence[seq];
            if (cmd == 0u) {
                drain_inbound();
                core_send_event();
            }
            if (!push(cmd)) {
                return false;
            }
            uint32_t response = 0;
            if (!pop(response)) {
                return false;
            }
            if (trace_count < 16u) {
                trace[trace_count++] = response;
            }
            if (response == cmd) {
                ++seq;
            } else {
                seq = 0;
                if (--attempts == 0u) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Core 1 into the ROM's wait loop through the power-on state
    /// machine: held in reset, released, and its "drained" word (a 0)
    /// popped. False when the word never came.
    static bool reset() {
        Psm::hold(PsmStage::proc1);
        // The read back both confirms the hold and fences the write out
        // of the bus bridge before the release below.
        for (uint32_t spins = 100'000u; (Psm::held() & PsmStage::proc1) == 0u; --spins) {
            if (spins == 0u) {
                return false;
            }
        }
        // Whatever core 1's old life left in this core's FIFO goes before
        // the 0 it pushes in its new one: held in reset, it can add
        // nothing now.
        drain_inbound();
        Psm::release(PsmStage::proc1);
        uint32_t word = 1;
        return pop(word) && word == 0u;
    }

    /// WHAT RUNS ON CORE 1 before the program's own entry, reached from
    /// the trampoline with the frame's two words in the argument
    /// registers: the per-core processor state the ROM's hand-over does
    /// not set (the file header), then the program.
    [[noreturn]] static void start(uint32_t entry, uint32_t vectors) {
#if defined(BRIO_RP2350_CORE_M33)
        SCB->VTOR = vectors;
        SCB->CPACR = SCB->CPACR | (0xFUL << 20);   // CP10, CP11: the FPU is per core
        __DSB();
        __ISB();
#else
        __asm__ volatile("csrw mtvec, %0" :: "r"(vectors) : "memory");
#endif
        entered_ = true;
        reinterpret_cast<void (*)()>(static_cast<uintptr_t>(entry))();
        // An entry that returns has no kernel to go back to.
        for (;;) {
            wait_for_interrupt();
        }
    }

private:
    static inline volatile bool entered_ = false;

    static void drain_inbound() {
        while ((SIO->FIFO_ST & SIO_FIFO_ST_VLD_BITS) != 0u) {
            (void)SIO->FIFO_RD;
        }
    }
    static bool push(uint32_t v) {
        for (uint32_t spins = 1'000'000u; spins != 0u; --spins) {
            if ((SIO->FIFO_ST & SIO_FIFO_ST_RDY_BITS) != 0u) {
                SIO->FIFO_WR = v;
                core_send_event();
                return true;
            }
        }
        return false;
    }
    static bool pop(uint32_t& v) {
        for (uint32_t spins = 1'000'000u; spins != 0u; --spins) {
            if ((SIO->FIFO_ST & SIO_FIFO_ST_VLD_BITS) != 0u) {
                v = SIO->FIFO_RD;
                return true;
            }
        }
        return false;
    }
};

/// The launch and the reset, under the name a program writes.
using Core1 = Core1Of<>;

/**
 * The mailbox FIFO (3.1.5), which on this target is the LAUNCH's channel
 * and nothing else - the bridge rings a doorbell instead (the file
 * header). It is offered because a program may want an ordered,
 * counted cross-core channel of its own, and because a bench suite has
 * to be able to say what the hardware is: 3.1.5 calls the FIFOs four
 * entries deep and the FIFO_ST register description calls them eight, so
 * the depth here is a MEASUREMENT and not a constant.
 *
 * Every verb is this core's own end: the outgoing FIFO is the one this
 * core writes, the incoming one the one it reads. Nothing here is used
 * by the bridge, and `Core1`'s protocol drives the same registers
 * directly, bounded.
 */
struct SioMailbox {
    SioMailbox() = delete;

    /// The line this core's incoming FIFO raises. brio never enables it
    /// - `Core1::protocol()` refuses to run while it is on, because its
    /// handler would eat the launch's echoes.
    static constexpr IRQn_Type irq() { return SIO_IRQ_FIFO_IRQn; }

    /// Room in the outgoing FIFO.
    static bool writable() { return (SIO->FIFO_ST & SIO_FIFO_ST_RDY_BITS) != 0u; }
    /// A word waiting in the incoming one.
    static bool readable() { return (SIO->FIFO_ST & SIO_FIFO_ST_VLD_BITS) != 0u; }

    /// One word to the other core, if there is room. False and nothing
    /// written otherwise - a write into a full FIFO changes no state but
    /// raises the sticky WOF.
    static bool push(uint32_t v) {
        if (!writable()) {
            return false;
        }
        SIO->FIFO_WR = v;
        core_send_event();
        return true;
    }

    /// One word out, if one is there.
    static bool pop(uint32_t& v) {
        if (!readable()) {
            return false;
        }
        v = SIO->FIFO_RD;
        return true;
    }

    /// Every word out of the incoming FIFO; how many there were.
    static uint32_t drain() {
        uint32_t n = 0;
        uint32_t word = 0;
        while (pop(word)) {
            ++n;
        }
        return n;
    }

    /// The two sticky misuse flags: a read of an empty incoming FIFO, a
    /// write to a full outgoing one.
    static bool read_on_empty() { return (SIO->FIFO_ST & SIO_FIFO_ST_ROE_BITS) != 0u; }
    static bool write_on_full() { return (SIO->FIFO_ST & SIO_FIFO_ST_WOF_BITS) != 0u; }
    /// Both cleared: any write to FIFO_ST does it.
    static void clear_errors() { SIO->FIFO_ST = SIO_FIFO_ST_ROE_BITS | SIO_FIFO_ST_WOF_BITS; }
};

} // namespace brio
