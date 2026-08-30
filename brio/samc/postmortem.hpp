/*
 * postmortem.hpp
 *
 * WHERE THE PROGRAM DIED, not just what killed it.
 *
 * The breadcrumb this stratum already has answers WHAT: kernel/panic.hpp
 * writes a PanicRecord - a code and a context byte - into the .noinit
 * storage samc/platform_sam.hpp hosts, samc/reset.hpp's `ResetReporter`
 * and `hard_fault_reset<P>()` end the program with a reset, and the next
 * boot reads the record back. What it cannot say is WHERE FROM: which
 * calls led into the wreck.
 *
 * The Cortex-M0+ answers that in hardware and for free. samc/mtb.hpp's
 * Micro Trace Buffer records every non-sequential change of the program
 * counter into ordinary SRAM with no CPU cycles spent, so at the moment
 * of the disaster the last N branches ARE in memory - they simply do not
 * survive, because the buffer is not checksummed, the trace keeps
 * running while the handler reads it, and nobody validates a word of it
 * at the next boot. This file is the glue that closes those three gaps:
 *
 *   1. FREEZE FIRST. `capture()` stops the trace before it does anything
 *      else - before the validity test, before the copy. The MTB traces
 *      the processor reading it, so a handler that decides, tests and
 *      prints first reads back its own branches and nothing else. The
 *      bench letter that measures this is the reason the verb exists.
 *   2. COPY, CHECKSUM, AND KEEP. The last `keep_packets` packets are
 *      copied out of the rolling buffer into a SEPARATE .noinit record
 *      with a magic word and a CRC-16 (util/crc.hpp) - so what survives
 *      the reset is a bounded, VALIDATED object, not a buffer somebody
 *      hopes was not touched. Table 18-1 promises nothing about SRAM
 *      through any reset, which is the same reason the PanicRecord
 *      carries a magic word.
 *   3. READ AND INVALIDATE. `take()` hands the packets over once and
 *      clears the record, so one crash is reported once.
 *
 * IT IS A SIBLING OF THE PANIC RECORD, NOT AN EXTENSION OF IT. The
 * kernel's PanicRecord is the kernel's; a trace is silicon this stratum
 * happens to have and the STM32 target may answer differently or not at
 * all. So the record lives here, in a samc type, and the two are read
 * side by side at boot.
 *
 * THE SAME RULE AS THE FAULT HANDLER'S: AN EXISTING RECORD IS NOT
 * OVERWRITTEN. samc/reset.hpp's hard_fault_reset() refuses to clobber a
 * PanicRecord because a fault after a panic is a CONSEQUENCE of
 * something already diagnosed; a second trace would bury the first for
 * exactly the same reason, and `capture()` returns false instead.
 *
 * THE TWO ENTRY PATHS, and which one really runs.
 *
 *     // orderly panic: capture, then whatever the next reporter does
 *     using Trace = brio::MtbPostMortem<256, 16>;
 *     using Reporter = brio::TracingReporter<Trace>;   // then ResetReporter
 *     brio::panic<brio::SamPlatform, Reporter>(code, context);
 *
 *     // a fault, which is also where an "orderly" panic ends up here
 *     extern "C" void HardFault_Handler() {
 *         brio::hard_fault_trace_reset<brio::SamPlatform, Trace>();
 *     }
 *
 * ON A BOARD WITH DHCSR.C_DEBUGEN CLEARED - which is what
 * `tools/bench.py` leaves after every SAM flash - THE SECOND PATH IS THE
 * ONE THAT RUNS EVEN FOR AN ORDERLY PANIC: panic() calls
 * P::break_here() BEFORE any reporter, break_here() is a BKPT, and a
 * BKPT with nothing halted on it escalates to HardFault. The
 * PanicRecord is already written by then (panic() writes it first), so
 * nothing is lost - but the `source` byte of the trace record says which
 * body wrote it, because on this core that is a property of the debug
 * state and not of the program. Bind both.
 *
 * COSTS NOTHING IT IS NOT ASKED FOR. The trace buffer is ordinary .bss
 * and the record ordinary .noinit; a program that never names the type
 * pays neither. `arm()` is a line of the app's own, so a program that
 * wants the trace only around one suspect region arms it there.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <span>

#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "samc/mtb.hpp"
#include "samc/reset.hpp"
#include "util/crc.hpp"

namespace brio {

/// Marks a trace record as valid - the MTB's own initials, and as
/// unlikely in cold RAM as the panic record's magic word.
inline constexpr uint32_t mtb_trace_magic = 0x4D544231UL;   // "MTB1"

/// The two `source` bytes this stratum uses. Any other value is the
/// caller's own: the field says WHO captured, and only the app knows
/// what bodies it bound.
inline constexpr uint8_t trace_from_fault = 1;   ///< the HardFault body
inline constexpr uint8_t trace_from_panic = 2;   ///< a panic Reporter

/// What `take()` hands over: the packets OLDEST FIRST, and who captured
/// them.
struct MtbTrace {
    std::span<const MtbPacket> packets;
    uint8_t source;
};

/**
 * The post-mortem trace store: a rolling MTB buffer, and a validated
 * copy of its tail that survives a reset.
 *
 * `trace_bytes` is the rolling buffer - a power of two of at least 16
 * bytes, aligned to its own size, which this type arranges. Bigger only
 * buys a longer window before the packets of interest are overwritten by
 * the program itself; it does not change WHICH packets are kept, since
 * the ones kept are always the last `keep_packets`.
 *
 * `keep_packets` is what crosses the reset, and it is the one number
 * worth thinking about: too few and the chain is cut off above the
 * fault, too many and .noinit grows for packets nobody reads. The bench
 * measures what a real chain costs - see docs/samc/mtb.md.
 */
template <uint32_t trace_bytes = 256, uint32_t keep_packets = 16>
struct MtbPostMortem {
    static_assert(keep_packets >= 1,
                  "a post-mortem that keeps no packets is not one");
    static_assert(Mtb::geometry_valid(trace_bytes),
                  "the MTB's trace buffer is a power of two of at least 16 "
                  "bytes (MASTER.MASK is log2(bytes) - 4)");
    static_assert(keep_packets <= Mtb::packets_for(trace_bytes),
                  "keeping more packets than the trace buffer holds: the "
                  "extra ones could never be filled");

    MtbPostMortem() = delete;

    static constexpr uint32_t buffer_bytes = trace_bytes;
    static constexpr uint32_t kept = keep_packets;

    /// The record as it sits in .noinit. Public because a boot-side
    /// report may want the raw count before deciding to take it; every
    /// field is meaningless unless `pending()`.
    struct Record {
        uint32_t magic;
        uint16_t crc;
        uint8_t count;    ///< packets actually copied, <= keep_packets
        uint8_t source;
        MtbPacket packets[keep_packets];
    };

    // ---- arming --------------------------------------------------------------

    /// Point the MTB at this store's buffer and start tracing. False
    /// only if the buffer could not be a trace buffer, which the
    /// static_asserts above have already made impossible - so a false
    /// here means the block refused, not that the geometry was wrong.
    static bool arm() {
        if (!Mtb::configure(buffer_, trace_bytes)) {
            return false;
        }
        Mtb::enable(true);
        return true;
    }

    /// Stop tracing and hand the block back. The record, if one has been
    /// captured, is untouched.
    static void disarm() { Mtb::release(); }

    static void* buffer() { return buffer_; }

    // ---- the capture ---------------------------------------------------------

    /**
     * FREEZE, COPY, CHECKSUM - in that order, and the order is the whole
     * point. Callable from a fault handler with interrupts dead: no
     * loop that is not bounded by `keep_packets`, no allocation, no
     * peripheral but the MTB's own registers.
     *
     * False, and NOTHING WRITTEN, when a valid record already stands:
     * the first diagnosis is the one worth keeping, the same rule
     * hard_fault_reset() applies to the PanicRecord. The trace is still
     * frozen, because stopping it is not the part that could be wrong.
     *
     * A record with a count of ZERO is a real answer and not a failure:
     * it says the capture ran and the buffer had nothing in it - the MTB
     * was never armed, or was stopped before the disaster.
     */
    static bool capture(uint8_t source) {
        (void)Mtb::freeze();
        if (pending()) {
            return false;
        }
        Record& r = record_;
        const uint32_t n = Mtb::snapshot(
            buffer_, trace_bytes, std::span<MtbPacket>(r.packets, keep_packets));
        r.count = static_cast<uint8_t>(n);
        r.source = source;
        r.crc = checksum(r);
        r.magic = mtb_trace_magic;   // last: a half-written record is not valid
        return true;
    }

    // ---- the boot side -------------------------------------------------------

    /// Whether a trace of a previous run is waiting: the magic word, a
    /// count the record could hold, and the checksum over both.
    static bool pending() {
        const Record& r = record_;
        return r.magic == mtb_trace_magic && r.count <= keep_packets &&
               r.crc == checksum(r);
    }

    /// A non-destructive view of the packets, oldest first. Empty unless
    /// pending().
    static std::span<const MtbPacket> packets() {
        if (!pending()) {
            return {};
        }
        return std::span<const MtbPacket>(record_.packets, record_.count);
    }

    /**
     * Read and invalidate: the packets and the source byte, once. The
     * span points into the record itself, which nothing writes again
     * until the next capture() - invalidating clears the magic word, not
     * the data - so it stays readable for as long as it takes to print.
     */
    static std::optional<MtbTrace> take() {
        if (!pending()) {
            return std::nullopt;
        }
        const MtbTrace t{std::span<const MtbPacket>(record_.packets,
                                                    record_.count),
                         record_.source};
        record_.magic = 0;
        return t;
    }

    /// Throw away whatever is there without reading it.
    static void clear() { record_.magic = 0; }

    /// The record as it stands, valid or not - for a boot-side report
    /// that wants to say WHY it refused one.
    static const Record& raw() { return record_; }

    /// CRC-16/CCITT-FALSE over the count, the source and exactly the
    /// packets the count claims. Deliberately not over the unused tail:
    /// a shorter trace must not be judged by bytes nobody wrote.
    static uint16_t checksum(const Record& r) {
        if (r.count > keep_packets) {
            return 0;
        }
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&r.count);
        const uint16_t len = static_cast<uint16_t>(
            2u + r.count * static_cast<uint16_t>(sizeof(MtbPacket)));
        return crc16(p, len);
    }

private:
    // Ordinary .bss: the rolling buffer does not have to survive
    // anything - the record does, and it is a checked copy. Aligned to
    // its own size because the write pointer wraps within a naturally
    // aligned region (samc/mtb.hpp).
    alignas(trace_bytes) static inline uint32_t buffer_[trace_bytes / 4]{};

    // NOTE: the same gcc 16 COMDAT-section quirk samc/platform_sam.hpp
    // records for its own .noinit variable - harmless, and the section
    // is right.
    [[gnu::section(".noinit")]] static inline Record record_;
};

/**
 * A panic Reporter that captures the trace and then does whatever the
 * next reporter does - by default samc/reset.hpp's `ResetReporter`, so
 * the pair is "freeze, keep the last branches, reset, report at the next
 * boot".
 *
 * Composition, not modification: reset.hpp's reporter is unchanged and
 * unaware, and any other reporter (an LED, a journal, a halt) can be the
 * `Next`. kernel/panic.hpp writes the PanicRecord BEFORE any reporter
 * runs, which is what makes chaining safe at all.
 */
template <typename Store, uint8_t source = trace_from_panic,
          typename Next = ResetReporter>
struct TracingReporter {
    static void report(PanicCode code, uint8_t context) {
        (void)Store::capture(source);
        Next::report(code, context);
    }
};

/**
 * The HardFault body with the trace taken first:
 *
 *     extern "C" void HardFault_Handler() {
 *         brio::hard_fault_trace_reset<brio::SamPlatform, Trace>();
 *     }
 *
 * The capture runs BEFORE hard_fault_reset<P>() - which writes the
 * PanicRecord if none stands and then resets - because the reset ends
 * the program and because every instruction spent before the freeze is
 * another packet over the evidence. Both halves keep an existing record:
 * a fault after a panic is a consequence of something already diagnosed.
 */
template <Platform P, typename Store>
[[noreturn]] void hard_fault_trace_reset(uint8_t context = 0) {
    (void)Store::capture(trace_from_fault);
    hard_fault_reset<P>(context);
}

} // namespace brio
