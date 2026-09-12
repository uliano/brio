/*
 * multicore.hpp
 *
 * The second core of the RP2040 as brio sees it (datasheet 2.3.1 the
 * SIO's CPUID and inter-core FIFOs, 2.3.2 the two NVICs, 2.8.2 the
 * bootrom's launch protocol, 2.13 the power-on state machine that can
 * hold one core in reset): the doorbell util/inbox.hpp rings, and the
 * launch of core 1.
 *
 * THE MODEL IS TWO KERNELS, NOT ONE ON TWO CORES (util/inbox.hpp's
 * header, design/kernel.md): every AO lives on one core, in the pack of
 * that core's kernel, on that core's platform type (Rp2040Platform<0>
 * or <1>); events cross through inboxes; a peripheral and its
 * interrupt line belong to the core whose kernel hosts the driver -
 * every line reaches BOTH NVICs (2.3.2), so the owning core is the one
 * that enables it, and a line enabled on both runs its handler twice.
 *
 * THE DOORBELL is the SIO's pair of inter-core FIFOs: eight words deep
 * each, one per direction, FIFO_WR feeding the OTHER core's FIFO_RD, and
 * one interrupt line per core - SIO_IRQ_PROC0 / SIO_IRQ_PROC1 - raised
 * while that core's inbound FIFO holds a word. `SioDoorbell<core>::ring()`
 * is called on the other core: one word in if there is room, nothing
 * if there is not (a bell is then already pending); `pop_all()` on the
 * receiving core empties the FIFO and clears the overflow flags; both
 * ends fence with the inbox's own barriers. The value of the word is
 * nothing: the bell is the interrupt.
 *
 * THE LAUNCH is the bootrom's protocol (2.8.2, the SDK's
 * multicore_launch_core1_raw): after a reset core 1 sits in the bootrom
 * draining its FIFO and echoing what it reads, and core 0 sends the
 * sequence 0, 0, 1, VTOR, SP, PC, each word answered by its echo, the
 * sequence restarted from the top on any wrong answer, a 0 preceded by
 * a drain of core 0's own inbound FIFO and an SEV. When the last word
 * is echoed core 1 loads the table, the stack and jumps: `entry` runs
 * on core 1 with interrupts masked, on the stack the linker script
 * reserves in the unstriped bank SRAM4 (`__stack_one_top`), the vector
 * table shared with core 0. THE LAUNCH RESETS CORE 1 FIRST: only a core
 * 1 in the bootrom's wait loop answers the protocol, and core 1 is not
 * there whenever core 0 alone was reset - SYSRESETREQ resets one core
 * (2.4.2, rp2040/reset.hpp), which is what a probe's `reset run` and a
 * program's Reset::core() do, and core 1 then keeps running its old
 * life (measured: deaf to the new core 0's launch). `Core1::reset()`
 * puts it back there through the power-on state machine's FRCE_OFF -
 * the processor subsystem whole, debug logic included - and pops the 0
 * core 1 pushes when it has drained its FIFO; `launch` then bounds
 * every wait and answers false when the protocol does not complete
 * (`protocol` alone is for a core 1 known to be waiting). The launch
 * uses the same FIFOs as the doorbell: it runs with core 0's doorbell
 * interrupt disabled (before it is enabled, or disabled around a
 * relaunch), and leaves both FIFOs empty.
 *
 * What core 1's entry does is the program's: its own ticker on its own
 * SysTick (CoreTicker<1>), the lines its drivers own enabled in ITS
 * NVIC, interrupts on, its kernel's run(). What it must not do: touch
 * the clock tree (core 0 owns it), or post to an AO of core 0 (the
 * queue refuses and counts).
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "rp2040/nvic.hpp"

/// The linker script's symbol for the top of core 1's stack (rp2040/ld/).
extern "C" char __stack_one_top;

namespace brio {

/// The bell towards `core`: the SIO FIFO that core reads, and its line.
template <uint8_t core>
struct SioDoorbell {
    static_assert(core < 2u, "the RP2040 has two cores, 0 and 1");
    SioDoorbell() = delete;

    static constexpr IRQn_Type irq() { return core == 0 ? SIO_IRQ_PROC0_IRQn : SIO_IRQ_PROC1_IRQn; }

    /// From the OTHER core: a bell in if there is room. A full FIFO
    /// holds eight bells already, and one is enough.
    static void ring() {
        if ((SIO->FIFO_ST & SIO_FIFO_ST_RDY_BITS) != 0u) {
            SIO->FIFO_WR = 1u;
            __SEV();
        }
    }

    /// On this core: every bell out, the overflow flags cleared.
    static void pop_all() {
        while ((SIO->FIFO_ST & SIO_FIFO_ST_VLD_BITS) != 0u) {
            (void)SIO->FIFO_RD;
        }
        SIO->FIFO_ST = SIO_FIFO_ST_ROE_BITS | SIO_FIFO_ST_WOF_BITS;
    }

    /// Whether a bell is pending for this core.
    static bool pending() { return (SIO->FIFO_ST & SIO_FIFO_ST_VLD_BITS) != 0u; }

    /// The line in THIS core's NVIC - so it is called on `core`.
    static void enable() { Nvic::enable(irq()); }
    static void disable() { Nvic::disable(irq()); }
};

/// Core 1: launched by core 0, put back into the bootrom by core 0.
struct Core1 {
    Core1() = delete;

    /// The stack the linker script reserves for core 1 (rp2040/ld/): the
    /// top of the unstriped bank SRAM4.
    static uint32_t default_stack_top() { return reinterpret_cast<uint32_t>(&__stack_one_top); }

    /// The words core 1 answered in the last launch, for a bench suite's
    /// eyes (at most the last sixteen).
    static inline uint32_t trace[16]{};
    static inline uint32_t trace_count = 0;

    /// Core 1 reset into the bootrom, then the protocol: `entry` runs on
    /// core 1 with `stack_top` as its stack pointer and this core's
    /// vector table. False when the reset went unanswered or the
    /// sequence did not complete within its bounded waits. Core 0's
    /// doorbell interrupt must be disabled while this runs.
    static bool launch(void (*entry)(), uint32_t stack_top = default_stack_top()) {
        return reset() && protocol(entry, stack_top);
    }

    /// The bootrom's launch protocol alone (the file header), for a core
    /// 1 known to be in the bootrom's wait loop: false when it did not
    /// answer the sequence within the bounded waits - it is not there
    /// (running: reset() first) or the FIFO is wedged.
    static bool protocol(void (*entry)(), uint32_t stack_top = default_stack_top()) {
        const uint32_t sequence[] = {
            0u, 0u, 1u, SCB->VTOR, stack_top, reinterpret_cast<uint32_t>(entry)};
        constexpr uint32_t words = sizeof sequence / sizeof sequence[0];
        uint32_t seq = 0;
        uint32_t attempts = 64;   // restarts of the sequence, bounded
        trace_count = 0;
        while (seq < words) {
            const uint32_t cmd = sequence[seq];
            if (cmd == 0u) {
                drain_inbound();
                __SEV();
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

    /// Core 1 into the bootrom's wait loop through the power-on state
    /// machine: held in reset, released, and its "drained" word (a 0)
    /// popped. False when the word never came.
    static bool reset() {
        hw_set(PSM->FRCE_OFF, PSM_FRCE_OFF_PROC1_BITS);
        for (uint32_t spins = 100'000u; (PSM->FRCE_OFF & PSM_FRCE_OFF_PROC1_BITS) == 0u; --spins) {
            if (spins == 0u) {
                return false;
            }
        }
        // Whatever core 1's old life rang into this core's FIFO (bells of
        // a kernel that was still running) goes before the 0 it pushes
        // in its new one: held in reset, it can add nothing now.
        drain_inbound();
        hw_clear(PSM->FRCE_OFF, PSM_FRCE_OFF_PROC1_BITS);
        uint32_t word = 1;
        return pop(word) && word == 0u;
    }

private:
    static void drain_inbound() {
        while ((SIO->FIFO_ST & SIO_FIFO_ST_VLD_BITS) != 0u) {
            (void)SIO->FIFO_RD;
        }
    }
    static bool push(uint32_t v) {
        for (uint32_t spins = 1'000'000u; spins != 0u; --spins) {
            if ((SIO->FIFO_ST & SIO_FIFO_ST_RDY_BITS) != 0u) {
                SIO->FIFO_WR = v;
                __SEV();
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

} // namespace brio
