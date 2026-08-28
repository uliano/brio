/*
 * borrowed.hpp
 *
 * The vocabulary of payloads that travel BY REFERENCE inside an event.
 *
 * Events are copied by value; a payload that must not be copied (a line
 * buffer, a DMA-sized transfer buffer) travels as a pointer, and a
 * pointer inside a copied event is a LOAN: someone still owns the
 * storage and will reuse it. brio admits exactly two loans - the payload
 * rule of docs/design/kernel.md, "borrow only what must be shared, and
 * every borrow is one of two":
 *
 *  - Lease::dispatch: valid only during the receiving AO's dispatch of
 *    that event. The lender may reuse the storage as soon as it runs
 *    again. Correct by construction when the borrower PRECEDES the
 *    lender in the Kernel pack (the kernel then serves the borrower
 *    before the lender is dispatched again - under a preemptive kernel
 *    the same ordering makes the borrower preempt the lender right at
 *    the post). The lender declares its borrowers with
 *    `using LendsTo = Subscribers<...>` and Kernel static_asserts the
 *    order (kernel.hpp).
 *  - Lease::reply: valid until the borrower sends back the completion
 *    event agreed by the protocol (BusDone for bus buffers). The lender
 *    keeps the storage alive and untouched until then. Ordering-
 *    independent; the reply IS the return of the loan.
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

#include <type_traits>

namespace brio {

/// How long a borrowed payload stays valid.
enum class Lease : unsigned char {
    dispatch,  ///< during the receiving dispatch only
    reply,     ///< until the borrower posts the agreed completion event
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

} // namespace brio
