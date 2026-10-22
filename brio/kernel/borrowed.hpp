/*
 * borrowed.hpp
 *
 * Pointers inside events, and how long what they point at stays valid.
 *
 * Events are copied by value. A buffer that must not be copied (a line
 * of received text, a transfer buffer a DMA channel fills) travels as a
 * pointer inside the event, and the event then carries a loan: the
 * sender still owns the storage and will reuse it. Borrowed<T, Lease>
 * is that pointer with the lifetime rule, the lease, written in its
 * type, so the rule is read at the field that declares the loan.
 *
 * The practical rules, one per lease:
 *
 *  - Lease::dispatch: the pointer is valid only while the receiving
 *    object's handler for that event runs. The receiver must not keep
 *    the pointer for a later event. The sender may reuse the storage the
 *    next time it runs, and that is safe only because the receiver runs
 *    first: the receiver must precede the sender in Tenuto's list of
 *    active objects. The sender declares its receivers with
 *    `using LendsTo = Subscribers<...>`, and Tenuto refuses at compile
 *    time a list in which a receiver follows the sender
 *    (kernel/tenuto.hpp). Under a preemptive kernel the same ordering
 *    would make the receiver preempt the sender right at the post.
 *  - Lease::reply: the pointer is valid until the receiver sends back
 *    the completion event the protocol agrees on (BusDone for a bus
 *    transfer). Until then the sender keeps the storage valid and follows
 *    the protocol's access rules (it must not write a buffer the bus is
 *    reading); after the completion event it may reuse the buffer. No
 *    ordering in the list is involved: the completion event is the
 *    return of the loan.
 *  - Lease::hold: not a payload but the event itself. A queue slot an
 *    object keeps past the dispatch of the event in it
 *    (kernel/event_queue.hpp's hold()), valid from that hold() until the
 *    same object's release() of it, in a later dispatch of its own. The
 *    lender is the object's own queue; a held slot is released by its
 *    holder and by nobody else, since another object releasing it would
 *    be a second consumer of the queue. `Held<E>` is the slot's number
 *    with this lease in its type (it pins the event type, not the
 *    holder; the rule keeps the holder). A holder that lends onward from
 *    the slot would do it as a `Borrowed<..., Lease::hold>`; no engine
 *    does, each keeping its own copy of the request.
 *
 * What the type does and does not enforce. Borrowed<T, L> is a plain
 * pointer: zero cost, trivially copyable, legal inside an event. The
 * compiler checks the dispatch ordering (above) and the type of what is
 * lent. It cannot stop a receiver from saving the raw pointer `.get()`
 * returns and using it after the lease ended; C++ has no borrow
 * checker, and staying inside the lease is the receiver's duty. The
 * type is also where a debug-build check would live (a lender epoch
 * compared on access); none is built.
 *
 * In use: a request descriptor names its loans in its fields. The tx,
 * rx and command buffers of a bus transfer and the source bytes of a
 * nonvolatile write are `Borrowed<..., Lease::reply>`, so "valid until
 * the reply lands" is read off the type instead of a comment. The engine
 * that walks such a buffer calls `.get()` once and indexes the raw
 * pointer: Borrowed is a view, not a container. The payload rule itself
 * (copy what is read once, borrow only what must be shared) is
 * docs/design/kernel.md's, "Payloads".
 *
 * `lend<L>(p)` names a loan at the call site, spelled like
 * `reply_to<Ao, P>()`: the lease is the explicit template argument, the
 * pointee type is deduced:
 *
 *     post<Bus>(Bus::Request{..., .tx = lend<Lease::reply>(buf), ...});
 *
 * A null loan is a default-constructed Borrowed: `{}` in a positional
 * list, or the field left out of a designated one. Lending a writable
 * buffer to a read-only field needs no ceremony: a loan converts to the
 * same loan over a more qualified pointee.
 */

#pragma once

#include <stdint.h>
#include <type_traits>

namespace brio {

/// How long a borrowed payload stays valid (the file header has the
/// rules).
enum class Lease : unsigned char {
    dispatch,  ///< during the receiving dispatch only
    reply,     ///< until the borrower posts the agreed completion event
    hold,      ///< until the holding AO releases the queue slot it holds
};

/// A pointer payload with its lease in the type. get() hands out the raw
/// pointer; using it past the lease is the caller's error, unchecked.
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
