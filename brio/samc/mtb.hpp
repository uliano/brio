/*
 * mtb.hpp
 *
 * The Cortex-M0+ Micro Trace Buffer (DS60001479M 10.3): a CoreSight
 * MTB-M0+ that records every non-sequential change of the program
 * counter as a pair of 32-bit words in SRAM, with no CPU cycles spent
 * and with write priority over the processor.
 *
 * THE INTERESTING CASE IS THE ONE WITH NO DEBUGGER. Every vendor
 * description of this block assumes a probe reading the buffer out
 * through the Debug Access Port. But the buffer is ORDINARY SRAM chosen
 * by software, so a program can point the MTB at a buffer of its own,
 * run, stop it, and READ ITS OWN TRACE - a hardware backtrace of the
 * last N branches, available after a fault, with no probe attached. That
 * is the shape this header builds and the bench suite proves.
 *
 * WHAT THE SILICON DOES.
 *
 * FOUR REGISTERS, AND THE DATA SHEET DESCRIBES NONE OF THEM. 10.3.2
 * names POSITION, MASTER, FLOW and BASE and then defers to the CoreSight
 * MTB-M0+ Technical Reference Manual, which is not among this project's
 * documents of record. The DEVICE HEADER is therefore the only local
 * authority on the bit layout and it is where every field below comes
 * from; anything the header does not name is measured or left alone.
 *
 * THE BUFFER IS A POWER OF TWO, ALIGNED TO ITSELF. MASTER.MASK holds
 * log2(bytes) - 4, so the smallest buffer is 16 bytes (two packets) and
 * the pointer wraps within a naturally aligned region of that size. A
 * buffer that is not aligned to its own size does not fail: it TRACES
 * SOMEWHERE ELSE, which is why `configure()` refuses one.
 *
 * POSITION AND FLOW HOLD OFFSETS FROM BASE, NOT ADDRESSES. BASE is a
 * read-only register giving where the SRAM the MTB writes to starts
 * (0x20000000 here); POSITION.POINTER and FLOW.WATERMARK are byte
 * offsets from it, with their low three bits implied zero because a
 * packet is eight bytes. So a buffer at 0x20001000 is programmed as
 * offset 0x1000, and this header does that subtraction rather than
 * making every caller remember it.
 *
 * A PACKET IS TWO WORDS: a source address and a destination address,
 * both with bit 0 carrying a flag whose NAMES belong to the TRM. This
 * header exposes the addresses with bit 0 masked off and the two flags
 * as unnamed bits, because inventing names for them would be stating
 * what is not enforced. What the bench establishes instead is what the
 * flags DO on this silicon - see docs/samc/mtb.md.
 *
 * WHAT NEEDS A DEBUGGER AND WHAT DOES NOT. MASTER.EN, MASTER.MASK,
 * FLOW.AUTOSTOP and the watermark are pure hardware and work with
 * nothing attached. MASTER.HALTREQ and FLOW.AUTOHALT ask the core to
 * HALT, which on ARMv6-M requires DHCSR.C_DEBUGEN - a bit
 * `tools/bench.py` deliberately CLEARS after every SAM flash, precisely
 * so a stray BKPT cannot stop an unattended board. They are exposed and
 * named; what they do here is measured rather than assumed.
 *
 * TWO EVENT USERS, AND THE TWO DOCUMENTS DISAGREE ABOUT THEIR NUMBERS.
 * MASTER.TSTARTEN and TSTOPEN gate the block's hardware trace-start and
 * trace-stop inputs, which on this family are wired to the event system.
 * Table 12-3 numbers those users 44 and 45; the device header's
 * EVENT_ID_USER_MTB_START / _STOP say 45 and 46. The constants below are
 * the HEADER's, per the house rule, and the bench settles which is
 * really connected.
 *
 * NO CLOCK OF ITS OWN. The MTB has no bit in any MCLK mask and no row
 * in table 12-3's clock columns: it is clocked with the processor and
 * there is nothing to enable.
 *
 * PAC. The MTB is write-protectable (`Pac::protect(Mtb::pac_id)`), and
 * ITS IDENTIFIER IS ONE THE DATA SHEET NEVER PRINTS: table 12-3's PAC
 * Index column is blank for this row while PAC.STATUSB and INTFLAGB both
 * carry an MTB bit and the header gives ID_MTB. See device_tables.hpp.
 *
 * NO ERRATA. DS80000740S has no MTB section.
 *
 * READING THE TAIL BACK IS A RACE AGAINST THE READER ITSELF. The MTB
 * traces the CPU that is reading it, so every branch the reading code
 * takes is another packet over the oldest thing still in the buffer.
 * `freeze()` is therefore the FIRST verb of any post-mortem path - not
 * an optimization but the difference between a trace and a picture of
 * the trace reader - and `snapshot()` is the bounded, allocation-free
 * copy of the last N packets a frozen buffer holds. Both are legal with
 * interrupts dead, which is what makes samc/postmortem.hpp possible.
 *
 * NOT BUILT (docs/samc/mtb.md carries the list): any decoder above the
 * packet pair - reconstructing a call chain wants the image's symbols
 * and belongs on the host; and the CoreSight management registers at
 * 0xF00 and above (claim tags, lock access, the authentication status),
 * which are a probe's business.
 */

#pragma once

#include <stdint.h>
#include <span>

#include "sam.h"

#include "samc/device_tables.hpp"

namespace brio {

/// One execution-trace packet as the MTB writes it: two 32-bit words.
///
/// Bit 0 of each word is a FLAG, not part of the address - every
/// instruction address on this core is halfword aligned, so bit 0 is
/// free. The CoreSight MTB-M0+ TRM names the two flags; that manual is
/// not a document of record here, so they are exposed as bits and
/// docs/samc/mtb.md says what they were measured doing.
struct MtbPacket {
    uint32_t source_word;
    uint32_t destination_word;

    /// Where the flow left: the address of the instruction that changed
    /// the PC non-sequentially.
    constexpr uint32_t source() const { return source_word & ~1UL; }
    /// Where it went.
    constexpr uint32_t destination() const { return destination_word & ~1UL; }

    constexpr bool source_flag() const { return (source_word & 1UL) != 0u; }
    constexpr bool destination_flag() const {
        return (destination_word & 1UL) != 0u;
    }
};

/// Everything MASTER and FLOW hold besides the buffer geometry.
struct MtbConfig {
    /// MASTER.TSTARTEN: let the hardware trace-start input (an event)
    /// set MASTER.EN.
    bool start_on_event = false;
    /// MASTER.TSTOPEN: let the hardware trace-stop input clear it.
    bool stop_on_event = false;
    /// FLOW.AUTOSTOP: stop tracing when the write pointer reaches the
    /// watermark, instead of wrapping over the oldest packets.
    bool auto_stop = false;
    /// FLOW.AUTOHALT: ask the core to HALT at the watermark. NEEDS A
    /// DEBUGGER - see this file's header.
    bool auto_halt = false;
    /// MASTER.SFRWPRIV / MASTER.RAMPRIV: restrict writes to the MTB's
    /// own registers, and to its SRAM, to privileged accesses. This core
    /// runs everything privileged in this framework, so both are
    /// vocabulary rather than protection here.
    bool registers_privileged = false;
    bool ram_privileged = false;
    /// Where in the buffer AUTOSTOP/AUTOHALT fire, as a packet index.
    /// `watermark_packets == 0` means "the end of the buffer".
    uint32_t watermark_packets = 0;
};

/**
 * The Micro Trace Buffer as a monostate resource.
 */
struct Mtb {
    Mtb() = delete;

    static constexpr uint16_t pac_id = mtb_pac_id();          // 36
    static constexpr uint8_t ev_user_start = mtb_start_user();
    static constexpr uint8_t ev_user_stop = mtb_stop_user();

    /// A packet is a pair of 32-bit words.
    static constexpr uint32_t packet_bytes = 8;

    static mtb_registers_t& regs() { return *MTB_REGS; }

    /// BASE: where the SRAM the MTB writes to begins. Read-only, and the
    /// origin POSITION and FLOW are measured from.
    static uint32_t sram_base() { return MTB_REGS->MTB_BASE; }

    // ---- geometry ------------------------------------------------------------

    /// MASTER.MASK for a buffer of `bytes`: log2(bytes) - 4. Returns 32
    /// - an impossible value the config check refuses - for anything
    /// that is not a power of two of at least 16 bytes.
    static constexpr uint8_t mask_for(uint32_t bytes) {
        if (bytes < 16u || (bytes & (bytes - 1u)) != 0u) {
            return 32u;
        }
        uint8_t m = 0;
        uint32_t n = bytes >> 4;
        while (n > 1u) {
            n >>= 1;
            ++m;
        }
        return m;
    }
    /// The inverse: how many bytes a MASK value spans.
    static constexpr uint32_t size_for(uint8_t mask) {
        return mask < 32u ? (16UL << mask) : 0u;
    }
    static constexpr uint32_t packets_for(uint32_t bytes) {
        return bytes / packet_bytes;
    }

    /// Whether a size and a watermark can describe a trace buffer at
    /// all, independent of where it sits: a power-of-two size of at
    /// least 16 bytes, and a watermark inside it (zero meaning "the
    /// end"). Constant-evaluable, so a caller's geometry can be settled
    /// at compile time.
    static constexpr bool geometry_valid(uint32_t bytes,
                                         uint32_t watermark_packets = 0) {
        return mask_for(bytes) < 32u && watermark_packets <= packets_for(bytes);
    }

    /// The same plus the PLACEMENT rules, which need the address: the
    /// buffer must be ALIGNED TO ITS OWN SIZE (the pointer wraps within
    /// a naturally aligned region, so a misaligned buffer does not fail
    /// - it traces somewhere else) and must lie above BASE.
    static bool buffer_valid(const void* buffer, uint32_t bytes) {
        if (!geometry_valid(bytes)) {
            return false;
        }
        const uint32_t addr = reinterpret_cast<uint32_t>(buffer);
        if ((addr & (bytes - 1u)) != 0u) {
            return false;
        }
        return addr >= sram_base();
    }

    // ---- configuration -------------------------------------------------------

    /**
     * Point the block at a buffer and write MASTER and FLOW - WITHOUT
     * enabling it. `enable(true)` is a separate verb so that the moment
     * tracing starts is a line of the caller's own, which is what makes
     * a trace window mean anything.
     *
     * The write pointer is reset to the start of the buffer and the WRAP
     * bit cleared, so a configure() is also the way to re-arm for a
     * second window.
     */
    static bool configure(void* buffer, uint32_t bytes,
                          const MtbConfig& cfg = MtbConfig{}) {
        if (!buffer_valid(buffer, bytes) ||
            !geometry_valid(bytes, cfg.watermark_packets)) {
            return false;
        }
        const uint32_t total = packets_for(bytes);
        MTB_REGS->MTB_MASTER = 0;   // stop before moving the pointer

        const uint32_t offset =
            reinterpret_cast<uint32_t>(buffer) - sram_base();
        MTB_REGS->MTB_POSITION = offset & MTB_POSITION_POINTER_Msk;

        const uint32_t wm_packets =
            cfg.watermark_packets == 0u ? total : cfg.watermark_packets;
        MTB_REGS->MTB_FLOW =
            ((offset + wm_packets * packet_bytes) & MTB_FLOW_WATERMARK_Msk) |
            (cfg.auto_stop ? MTB_FLOW_AUTOSTOP_Msk : 0u) |
            (cfg.auto_halt ? MTB_FLOW_AUTOHALT_Msk : 0u);

        MTB_REGS->MTB_MASTER =
            MTB_MASTER_MASK(mask_for(bytes)) |
            (cfg.start_on_event ? MTB_MASTER_TSTARTEN_Msk : 0u) |
            (cfg.stop_on_event ? MTB_MASTER_TSTOPEN_Msk : 0u) |
            (cfg.registers_privileged ? MTB_MASTER_SFRWPRIV_Msk : 0u) |
            (cfg.ram_privileged ? MTB_MASTER_RAMPRIV_Msk : 0u);
        return true;
    }

    /// MASTER.EN. Nothing else in MASTER is disturbed, so a stopped
    /// trace can be resumed exactly where it left off.
    static void enable(bool on) {
        const uint32_t m = MTB_REGS->MTB_MASTER;
        MTB_REGS->MTB_MASTER = on ? (m | MTB_MASTER_EN_Msk)
                                  : (m & ~MTB_MASTER_EN_Msk);
    }
    static bool enabled() {
        return (MTB_REGS->MTB_MASTER & MTB_MASTER_EN_Msk) != 0u;
    }

    /// Stop and hand the block back: tracing off, MASTER and FLOW
    /// cleared to their reset values.
    static void release() {
        MTB_REGS->MTB_MASTER = 0;
        MTB_REGS->MTB_FLOW = 0;
    }

    static uint32_t master() { return MTB_REGS->MTB_MASTER; }
    static uint32_t flow() { return MTB_REGS->MTB_FLOW; }
    static uint32_t position() { return MTB_REGS->MTB_POSITION; }

    /// MASTER.MASK as it currently reads.
    static uint8_t mask() {
        return static_cast<uint8_t>((master() & MTB_MASTER_MASK_Msk) >>
                                    MTB_MASTER_MASK_Pos);
    }

    // ---- reading the trace back ---------------------------------------------

    /// The byte offset from BASE the next packet will be written at.
    static uint32_t write_offset() {
        return position() & MTB_POSITION_POINTER_Msk;
    }

    /// POSITION.WRAP: the pointer has been round at least once, so the
    /// oldest packets have been overwritten and the buffer is full.
    static bool wrapped() {
        return (position() & MTB_POSITION_WRAP_Msk) != 0u;
    }

    /**
     * How many packets a window produced, for a buffer that has NOT
     * wrapped: the distance from the buffer's own offset to the write
     * pointer. Once WRAP is set the answer is simply the whole buffer
     * and the oldest packet is the one the pointer is about to
     * overwrite, so a caller reading a wrapped buffer walks it from
     * `write_offset()` round to itself.
     */
    static uint32_t packets_written(const void* buffer, uint32_t bytes) {
        if (wrapped()) {
            return packets_for(bytes);
        }
        const uint32_t start =
            reinterpret_cast<uint32_t>(buffer) - sram_base();
        const uint32_t now = write_offset();
        return now >= start ? (now - start) / packet_bytes : 0u;
    }

    /// Packet `index` of a trace buffer, read straight out of SRAM.
    static MtbPacket packet(const void* buffer, uint32_t index) {
        const volatile uint32_t* w =
            static_cast<const volatile uint32_t*>(buffer) + index * 2u;
        return MtbPacket{w[0], w[1]};
    }

    // ---- the post-mortem pair ------------------------------------------------

    /**
     * STOP THE TRACE, AND STOP IT FIRST. Clears MASTER.EN and leaves
     * everything else in MASTER alone; returns whether the trace was
     * running.
     *
     * This is `enable(false)` under a name that says why it is the first
     * line of a fault handler rather than a step in one. The MTB traces
     * the processor that reads it, so every branch spent deciding to
     * read - the handler's own prologue, a validity test, a print - is
     * another packet written over the oldest packet still there. Read a
     * running buffer and what comes back is the reader's own history;
     * the suite measures exactly that.
     *
     * Nothing here waits, allocates, or takes a lock: one load, one
     * store. Legal with interrupts dead and legal from a fault handler.
     */
    [[gnu::always_inline]] static bool freeze() {
        const uint32_t m = MTB_REGS->MTB_MASTER;
        MTB_REGS->MTB_MASTER = m & ~MTB_MASTER_EN_Msk;
        return (m & MTB_MASTER_EN_Msk) != 0u;
    }

    /**
     * Copy the LAST packets the buffer holds into `out`, OLDEST FIRST,
     * and return how many were copied.
     *
     * The write pointer names the NEXT slot, so the newest packet is the
     * one before it and the walk runs backwards from there - modulo the
     * buffer when POSITION.WRAP says the pointer has been round, which
     * is the only case where the oldest surviving packet is not the
     * first slot. A buffer that has not wrapped yet simply holds fewer
     * packets than asked for and the answer is short; a caller reads the
     * count, not the span's size.
     *
     * `out` is the caller's storage and is never exceeded: with room for
     * fewer packets than the buffer holds, the OLDEST are the ones
     * dropped, which is the right end to lose in a post-mortem.
     *
     * Bounded and allocation-free - `out.size()` iterations of two loads
     * - and it does not touch the block's registers except to read
     * POSITION. It does NOT stop the trace: call freeze() first, or
     * measure the reader instead of the program.
     *
     * Zero for a span with no room, for a geometry that is not a trace
     * buffer's, and for a POSITION that does not point inside `buffer`
     * at all - the last being the "somebody else's buffer" case, which
     * is worth a zero rather than a wild read.
     */
    static uint32_t snapshot(const void* buffer, uint32_t bytes,
                             std::span<MtbPacket> out) {
        if (out.empty() || !geometry_valid(bytes)) {
            return 0;
        }
        const uint32_t start =
            reinterpret_cast<uint32_t>(buffer) - sram_base();
        const uint32_t now = write_offset();
        if (now < start || now >= start + bytes) {
            return 0;
        }
        const uint32_t total = packets_for(bytes);
        const uint32_t next = (now - start) / packet_bytes;
        const uint32_t available = wrapped() ? total : next;
        const uint32_t room = static_cast<uint32_t>(out.size());
        const uint32_t n = available < room ? available : room;
        if (n == 0u) {
            return 0;
        }
        uint32_t index = (next + total - n) % total;
        for (uint32_t i = 0; i < n; ++i) {
            out[i] = packet(buffer, index);
            index = index + 1u == total ? 0u : index + 1u;
        }
        return n;
    }
};

} // namespace brio
