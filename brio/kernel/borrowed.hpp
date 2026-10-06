/*
 * borrowed.hpp
 *
 * The vocabulary of payloads that travel BY REFERENCE inside an event.
 *
 * Events are copied by value; a payload that must not be copied (a line
 * buffer, a DMA-sized transfer buffer) travels as a pointer, and a
 * pointer inside a copied event is a LOAN: someone still owns the
 * storage and will reuse it. brio admits exactly three loans - two for
 * payloads, the payload rule of docs/design/kernel.md, "borrow only what
 * must be shared, and every borrow is one of two", and one for the event
 * itself, the queue slot an AO holds:
 *
 *  - Lease::dispatch: valid only during the receiving AO's dispatch of
 *    that event. The lender may reuse the storage as soon as it runs
 *    again. Correct by construction when the borrower PRECEDES the
 *    lender in the Tenuto pack (the kernel then serves the borrower
 *    before the lender is dispatched again - under a preemptive kernel
 *    the same ordering makes the borrower preempt the lender right at
 *    the post). The lender declares its borrowers with
 *    `using LendsTo = Subscribers<...>` and Tenuto static_asserts the
 *    order (tenuto.hpp).
 *  - Lease::reply: valid until the borrower sends back the completion
 *    event agreed by the protocol (BusDone for bus buffers). The lender
 *    keeps the storage alive and untouched until then. Ordering-
 *    independent; the reply IS the return of the loan.
 *  - Lease::hold: a queue slot an AO keeps past the dispatch of the event
 *    in it (kernel/event_queue.hpp's hold()), valid from that hold()
 *    until the SAME AO's release() of it, which it issues in a later
 *    dispatch of its own. The lender is the AO's own queue and the
 *    borrower the AO: a held slot is released by its owner and by nobody
 *    else - another AO releasing it would be a second consumer of the
 *    queue. `Held<E>` is the slot's number with this lease in its type
 *    - it pins the event type, not the owner, which is the rule's to
 *    keep. A holder that lends onward from the slot (an engine reading a
 *    request for a whole tenure) would do it as a
 *    `Borrowed<..., Lease::hold>`; no engine does today - each keeps
 *    its own copy, its family's choice to revisit.
 *
 * Borrowed<T, L> is a plain pointer with the lease written in its type:
 * zero cost, trivially copyable (it lives inside events), and it makes
 * the contract readable at the field where the loan is declared. It
 * cannot stop a receiver from stashing the raw pointer past its window
 * (C++ has no borrow checker); what it CAN do is name the rule at the
 * point of use, and host a debug-build epoch check the day a test needs
 * one (planned, not built: an 8-bit lender epoch compared on access,
 * panic on a stale loan).
 *
 * A request descriptor names its loans in its FIELDS: the tx/rx/cmd
 * buffers of a bus transfer and the source bytes of a nonvolatile write
 * are all `Borrowed<..., Lease::reply>`, so the rule ("valid until the
 * reply lands") is read off the type instead of a comment. The engine
 * that walks such a buffer calls `.get()` once and indexes the raw
 * pointer: Borrowed is a view, not a container.
 *
 * `lend<L>(p)` is the maker, deliberately spelled like `reply_to<Ao, P>()`
 * - the lease is the explicit template argument, the pointee type is
 * deduced:
 *
 *     post<Bus>(Bus::Request{..., .tx = lend<Lease::reply>(buf), ...});
 *
 * A null loan is a default-constructed Borrowed: `{}` in a positional
 * list, or simply the field left out of a designated one. Lending a
 * writable buffer to a read-only field needs no ceremony either - a loan
 * converts to the same loan over a more qualified pointee.
 */

#pragma once

#include <stdint.h>
#include <type_traits>

namespace brio {

/// How long a borrowed payload stays valid.
enum class Lease : unsigned char {
    dispatch,  ///< during the receiving dispatch only
    reply,     ///< until the borrower posts the agreed completion event
    hold,      ///< until the holding AO releases the queue slot it holds
};

/// A pointer payload with its lease in the type. See the header comment.
template <typename T, Lease L>
class Borrowed {
public:
    static constexpr Lease lease = L;

    constexpr Borrowed() = default;                     // null loan
    constexpr explicit Borrowed(T* p) : p_(p) {}

    /// A loan of the SAME lease over a less qualified pointee converts
    /// in: lending a writable buffer as read-only bytes is the ordinary
    /// case (a tx buffer the app fills, a field that only reads it), and
    /// without this every such call site would have to spell the pointee
    /// type. Only qualification conversions pass - U* to T* must be
    /// implicit, so nothing derived-to-base or unrelated slips through.
    template <typename U>
        requires (!std::is_same_v<U, T> && std::is_convertible_v<U*, T*>)
    constexpr Borrowed(Borrowed<U, L> other) : p_(other.get()) {}

    constexpr T* get() const { return p_; }
    constexpr T& operator*() const { return *p_; }
    constexpr T* operator->() const { return p_; }
    constexpr explicit operator bool() const { return p_ != nullptr; }

private:
    T* p_ = nullptr;
};

/// Name a loan at the call site: `lend<Lease::reply>(buf)`. The lease is
/// spelled, the pointee type is deduced.
template <Lease L, typename T>
constexpr Borrowed<T, L> lend(T* p) {
    return Borrowed<T, L>{p};
}

/// A queue slot an AO holds: the slot's number with Lease::hold in its
/// type, the handle kernel/event_queue.hpp's hold() returns and its at()
/// and release() take back. One byte; slot numbers start at 1, so a
/// default-constructed Held - zero - is null (holds nothing), and a
/// static array of them is .bss, not an image in flash.
template <typename E>
class Held {
public:
    static constexpr Lease lease = Lease::hold;

    constexpr Held() = default;                        // null: holds nothing
    /// The slot numbered `handle`; 0 (a queue's none) makes a null Held.
    constexpr explicit Held(uint8_t handle) : handle_(handle) {}

    /// The slot's number, 1 and up; 0 on a null Held.
    constexpr uint8_t handle() const { return handle_; }
    constexpr explicit operator bool() const { return handle_ != 0u; }
    constexpr bool operator==(const Held&) const = default;

private:
    uint8_t handle_ = 0;
};

} // namespace brio
