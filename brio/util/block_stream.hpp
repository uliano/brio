/*
 * block_stream.hpp (util)
 *
 * The vocabulary of BLOCK STREAMS - data that moves in caller-owned
 * buffers at a rate the CPU never touches per sample - and BlockRelay,
 * the active object that hands filled blocks to subscribers as loans.
 *
 * THE CONTRACT IS ABOUT BLOCKS, NOT ABOUT DMA. What the concepts below
 * name is a shape: a source that fills one caller-owned buffer while the
 * caller drains another and accounts for what it could not keep, and a
 * player that feeds one caller-owned table to a peripheral for ever. On
 * the SAM C21 both are DMA engines (samc/dmac.hpp's DmaPingPongEngine
 * and DmaLoopEngine); on a machine with no DMA the same concepts are
 * satisfiable by an interrupt handler filling the buffers - nothing here
 * asks HOW a block gets full. The concepts exist BEFORE their second
 * implementation on purpose: they are the fixed point against which the
 * next platform's stream machinery is measured, so that friction shows
 * up as "this concept does not fit" rather than as silent divergence.
 *
 * WHY AN AO AT ALL, AND WHY IT IS NOT A SAMPLER. MeterSampler exists to
 * DISCARD: a meter's latest value is the only one worth publishing, so
 * it paces publication below the capture rate and stale readings die in
 * the latch. A block stream is the opposite economy - EVERY block must
 * reach its consumer exactly once, because the samples inside it exist
 * nowhere else. So BlockRelay is event-driven, not paced: the source's
 * own completion is the wakeup (the ISR glue posts BlockDone), the
 * source's two buffers are the slack that absorbs dispatch latency, and
 * a consumer that falls behind shows up in the SOURCE's overrun count -
 * the engine skips laps rather than tear blocks, and the relay invents
 * no second truth beside it.
 *
 * THE LOAN DISCIPLINE, which is the whole design. A block is too big to
 * copy through a queue, so BlockReady carries a Lease::dispatch loan
 * (kernel/borrowed.hpp): valid during the receiving dispatch only. The
 * relay declares `LendsTo = Subs` and the kernel refuses a pack where a
 * borrower does not precede it - which is what makes the release timing
 * correct BY CONSTRUCTION: when a BlockRelay dispatch starts, the kernel
 * has already served every borrower's queue, so every block lent in the
 * PREVIOUS dispatch has been consumed, and releasing it back to the
 * source is the first thing the new dispatch does (SerialPort's
 * two-line-buffer contract, generalized). Between the lend and that
 * release the source's other buffer is the one being filled, so the
 * consumer reads memory nothing is writing.
 *
 * WHAT IS DELIBERATELY ABSENT. A playback AO: a player needs no pacing
 * (the peripheral's own request paces it) and nothing to publish per
 * lap; its laps()/faults() are a readback, and an owner that wants them
 * as events arms its own TimeEvent. One-shot burst vocabulary: a finite
 * capture is a source started and stopped - or, below util, a bare
 * engine block - and earns a concept only when a second user shapes it.
 * Restart policy after a stall: release() restarting the stream is the
 * SOURCE's contract; whether the gap warrants an app-level response is
 * the subscriber's business, told by the overrun count it can read.
 *
 * Validated on: SAM C21 (the two DMA streaming engines) and the host
 * fake. (docs/design/block-stream.md.)
 */

#pragma once

#include <stdint.h>
#include <concepts>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"

namespace brio {

/**
 * What BlockRelay needs of a source: caller-owned filled blocks handed
 * over whole, given back with release(), and the honest accounting -
 * laps filled, laps skipped because the caller held both buffers, and
 * whether it is stalled right now. `element` is the sample type; ready()
 * hands the block back as pointer-to-volatile because the thing that
 * filled it is invisible to the compiler (a DMA controller today, an
 * interrupt tomorrow), and a plain implementation converts in for free.
 */
template <typename S>
concept BlockSource = requires {
    typename S::element;
    { S::ready() } -> std::convertible_to<const volatile typename S::element*>;
    { S::ready_length() } -> std::convertible_to<uint16_t>;
    { S::release() } -> std::convertible_to<bool>;
    { S::laps() } -> std::convertible_to<uint32_t>;
    { S::overruns() } -> std::convertible_to<uint32_t>;
    { S::stalled() } -> std::convertible_to<bool>;
};

/**
 * The playback half of the vocabulary: one caller-owned table fed to a
 * peripheral for ever. No AO consumes it - the readbacks are the
 * surface, and laps() moving is the one fact that says the stream is
 * alive. Named here so the NEXT platform's playback machinery has a
 * contract to meet, exactly as BlockSource does for capture.
 */
template <typename Pl>
concept BlockPlayer = requires {
    typename Pl::element;
    { Pl::laps() } -> std::convertible_to<uint32_t>;
    { Pl::faults() } -> std::convertible_to<uint32_t>;
    { Pl::running() } -> std::convertible_to<bool>;
    Pl::stop();
};

/// Posted to BlockRelay - by the completion glue (a block just filled)
/// or by the relay to itself (loans to return, blocks still waiting).
/// Carrying no source index is deliberate: a dispatch scans every
/// source, so a coalesced or dropped wakeup delays work and never loses
/// it.
struct BlockDone {};

/// Published per filled block: which source (its position in the
/// relay's list), the block as a Lease::dispatch loan, and how many
/// elements it holds. Read it during the receiving dispatch; the relay
/// hands the buffer back to the source right after.
template <typename T>
struct BlockReady {
    uint8_t source;
    Borrowed<const volatile T, Lease::dispatch> data;
    uint16_t length;
};

/**
 * The AO: woken by completions, it returns the loans of its previous
 * dispatch to their sources, then lends ONE ready block per source to
 * the subscribers - and re-posts itself whenever it lent anything, so
 * the returning dispatch is guaranteed even when the stream has gone
 * quiet (a stalled source emits no further completions; the self-post
 * is what lets release() restart it).
 *
 * The relay configures no hardware and starts no stream - it does not
 * know any. The application arms its engines, binds the completion
 * glue (`post<Relay>(BlockDone{})` after the engine's complete()), and
 * lists the relay AFTER its subscribers in the kernel pack; the
 * static_assert on LendsTo holds it to that.
 */
template <Platform P, typename Subs, BlockSource... Sources>
    requires (sizeof...(Sources) > 0 && sizeof...(Sources) <= 8)
class BlockRelay : public Fsm<BlockRelay<P, Subs, Sources...>, BlockDone> {
    using Self = BlockRelay<P, Subs, Sources...>;
    using Base = Fsm<Self, BlockDone>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    static constexpr uint8_t source_count = sizeof...(Sources);

    /// Depth: one completion per source can land before the relay runs,
    /// plus its own self-post. An overflow is harmless by construction -
    /// any queued BlockDone triggers a full scan - and the queue's
    /// saturating counter still reports it.
    static inline EventQueue<Event, source_count + 2, P> queue;

    /// BlockReady is a Lease::dispatch loan: every subscriber must
    /// precede this AO in the pack (kernel.hpp refuses otherwise).
    using LendsTo = Subs;

    static void init() {
        lent_ = 0;
        published_ = 0;
        Base::start(&running);
    }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Blocks published since init(), all sources together. Saturating.
    static uint16_t published() { return published_; }

    /// A source's own accounting, passed through by list position - the
    /// relay keeps no second truth beside the source's.
    static uint32_t laps(uint8_t index) {
        uint32_t out = 0;
        uint8_t k = 0;
        (pick_laps<Sources>(k, index, out), ...);
        return out;
    }
    static uint32_t overruns(uint8_t index) {
        uint32_t out = 0;
        uint8_t k = 0;
        (pick_overruns<Sources>(k, index, out), ...);
        return out;
    }

private:
    static Status running(const Event& e) {
        return match(e,
            [](Entry) { return Base::handled(); },
            [](Exit) { return Base::handled(); },
            [](BlockDone) {
                // 1. Every block lent in the previous dispatch has been
                //    consumed (the pack order guarantees the borrowers
                //    ran first): give the buffers back. For a stalled
                //    source this release is also the restart.
                uint8_t i = 0;
                (return_one<Sources>(i), ...);
                lent_ = 0;
                // 2. Lend one ready block per source. One, not both a
                //    stall can hold: the second needs this dispatch's
                //    loan returned first, and the self-post below is
                //    what comes back for it.
                i = 0;
                (lend_one<Sources>(i), ...);
                if (lent_ != 0) {
                    post<Self>(BlockDone{});
                }
                return Base::handled();
            });
    }

    template <typename S>
    static void return_one(uint8_t& i) {
        if (lent_ & static_cast<uint8_t>(1u << i)) {
            (void)S::release();
        }
        ++i;
    }

    template <typename S>
    static void lend_one(uint8_t& i) {
        const volatile typename S::element* p = S::ready();
        if (p != nullptr) {
            publish(Subs{}, BlockReady<typename S::element>{
                                i, lend<Lease::dispatch>(p), S::ready_length()});
            lent_ |= static_cast<uint8_t>(1u << i);
            if (published_ != UINT16_MAX) {
                ++published_;
            }
        }
        ++i;
    }

    /// The pack unrolled into a chain of compares - the MeterSampler
    /// shape (the sources are monostates whose constructors are deleted,
    /// so the pick is by type, never by value).
    template <typename S>
    static void pick_laps(uint8_t& k, uint8_t index, uint32_t& out) {
        if (k++ == index) {
            out = S::laps();
        }
    }

    template <typename S>
    static void pick_overruns(uint8_t& k, uint8_t index, uint32_t& out) {
        if (k++ == index) {
            out = S::overruns();
        }
    }

    static inline uint8_t lent_ = 0;
    static inline uint16_t published_ = 0;
};

} // namespace brio
