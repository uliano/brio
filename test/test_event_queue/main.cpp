// Host tests for brio::EventQueue (the per-AO index carousel) on
// HostPlatform: the producers' push, the consumer's take/at/release, the
// hold, the overflow moment, the all-zero static object - and the
// INTERLEAVING EXPLORER, a scripted "ISR" that pushes at every boundary
// between two of the consumer's shared accesses (the queue calls the
// platform's interleave_point() there), at one boundary and then at every
// pair, each run checked against a model of what the queue must do.
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <stdint.h>
#include <string>
#include <variant>
#include <vector>

#include "kernel/event_queue.hpp"
#include "host/platform.hpp"

using brio::EventQueue;
using brio::Held;
using brio::HostPlatform;

namespace {

struct ButtonDown { uint8_t id; };
struct TempReading { int16_t centi; };
struct Tick {};
using Event = std::variant<Tick, ButtonDown, TempReading>;

/// Pushes until one is refused; the count accepted. Leaves the queue full.
template <typename Q, typename E>
uint16_t fill(Q& q, E value) {
    uint16_t accepted = 0;
    for (;;) {
        const uint16_t before = q.overflows();
        q.push(value);
        if (q.overflows() != before) {
            return accepted;
        }
        ++accepted;
    }
}

} // namespace

// ---------------------------------------------------------------------------
// The producers' side and the copy-out drain
// ---------------------------------------------------------------------------

TEST_CASE("a fresh queue is empty") {
    HostPlatform::reset();
    EventQueue<uint8_t, 4, HostPlatform> q;

    CHECK(q.empty());
    CHECK(q.size() == 0);
    CHECK(q.capacity() == 4);
    CHECK(q.overflows() == 0);
    CHECK_FALSE(q.pop().has_value());
    CHECK(q.take() == q.none);
}

TEST_CASE("push/pop preserves FIFO order") {
    HostPlatform::reset();
    EventQueue<uint8_t, 3, HostPlatform> q;

    q.push(10);
    q.push(20);
    q.push(30);
    CHECK(q.size() == 3);
    CHECK(q.overflows() == 0);

    CHECK(q.pop().value() == 10);
    CHECK(q.pop().value() == 20);
    CHECK(q.pop().value() == 30);
    CHECK(q.empty());
}

TEST_CASE("overflow drops the new event, counts, and leaves content intact") {
    HostPlatform::reset();
    EventQueue<uint8_t, 2, HostPlatform> q;   // three slots

    q.push(1);
    q.push(2);
    q.push(3);            // the spare slot: nothing is being dispatched
    q.push(4);            // every slot occupied: dropped
    q.push(5);            // dropped

    CHECK(q.size() == 3);
    CHECK(q.overflows() == 2);
    CHECK(q.pop().value() == 1);   // survivors are the OLD events, in order
    CHECK(q.pop().value() == 2);
    CHECK(q.pop().value() == 3);
    CHECK_FALSE(q.pop().has_value());

    q.push(6);            // queue works normally after the overflow
    CHECK(q.pop().value() == 6);
    CHECK(q.overflows() == 2);     // the counter is history, not state
}

TEST_CASE("indices wrap correctly over many cycles at arbitrary depth") {
    HostPlatform::reset();
    EventQueue<uint16_t, 5, HostPlatform> q;   // 5: deliberately not a power of 2

    // keep 2 events waiting while cycling far past every position's wrap
    q.push(0);
    q.push(1);
    for (uint16_t i = 2; i < 1000; ++i) {
        q.push(i);
        CHECK(q.pop().value() == i - 2);
    }
    CHECK(q.size() == 2);
    CHECK(q.overflows() == 0);
}

TEST_CASE("the overflow counter saturates instead of wrapping") {
    HostPlatform::reset();
    EventQueue<uint8_t, 1, HostPlatform> q;   // two slots

    q.push(42);
    q.push(43);
    for (uint32_t i = 0; i < 70000; ++i) {  // > UINT16_MAX overflows
        q.push(0);
    }
    CHECK(q.overflows() == UINT16_MAX);
    CHECK(q.pop().value() == 42);
    CHECK(q.pop().value() == 43);
}

TEST_CASE("a variant event survives the queue with alternative and payload") {
    HostPlatform::reset();
    EventQueue<Event, 4, HostPlatform> q;
    static_assert(sizeof(Event) <= 8, "event size budget");

    q.push(Tick{});
    q.push(ButtonDown{7});
    q.push(TempReading{-1250});

    CHECK(std::holds_alternative<Tick>(q.pop().value()));

    Event e = q.pop().value();
    const auto* button = std::get_if<ButtonDown>(&e);
    CHECK(button != nullptr);
    if (button != nullptr) {
        CHECK(button->id == 7);
    }

    e = q.pop().value();
    const auto* temp = std::get_if<TempReading>(&e);
    CHECK(temp != nullptr);
    if (temp != nullptr) {
        CHECK(temp->centi == -1250);
    }
}

TEST_CASE("every operation balances its critical section") {
    HostPlatform::reset();
    EventQueue<uint8_t, 2, HostPlatform> q;

    q.push(1);
    q.push(2);
    q.push(3);
    q.push(4);            // overflow path too
    (void)q.pop();
    const uint8_t h = q.take();
    (void)q.hold();
    q.release(h);
    (void)q.empty();
    (void)q.size();
    (void)q.overflows();

    CHECK(HostPlatform::CriticalSection::depth == 0);
}

// ---------------------------------------------------------------------------
// The consumer: take() hands over a slot's number, at() the event in it,
// the slot outstanding until released - and none of it masks.
// ---------------------------------------------------------------------------

TEST_CASE("take() on an empty queue is none and takes no critical section") {
    HostPlatform::reset();
    EventQueue<Event, 3, HostPlatform> q;

    CHECK(q.take() == q.none);
    CHECK(q.empty());
    CHECK(q.size() == 0);
    CHECK(HostPlatform::CriticalSection::entries == 0);
}

TEST_CASE("the consumer never masks: take, at, hold, release and the probes") {
    HostPlatform::reset();
    EventQueue<Event, 3, HostPlatform> q;
    q.push(ButtonDown{1});
    q.push(ButtonDown{2});
    CHECK(HostPlatform::CriticalSection::entries == 2);   // one per push

    const uint8_t a = q.take();
    REQUIRE(a != q.none);
    CHECK(std::get<ButtonDown>(q.at(a)).id == 1);
    CHECK(q.size() == 1);            // the taken event is not waiting
    const Held<Event> held = q.hold();
    CHECK(q.take_hold());
    const uint8_t b = q.take();
    REQUIRE(b != q.none);
    CHECK(std::get<ButtonDown>(q.at(b)).id == 2);
    CHECK_FALSE(q.take_hold());
    q.release(b);
    q.release(held);
    CHECK(q.take() == q.none);
    CHECK(q.empty());
    CHECK(q.size() == 0);
    (void)q.pop();
    CHECK(HostPlatform::CriticalSection::entries == 2);
}

TEST_CASE("while an event is dispatched, depth more can wait and its slot is never written") {
    HostPlatform::reset();
    EventQueue<Event, 3, HostPlatform> q;
    q.push(ButtonDown{7});

    const uint8_t served = q.take();
    REQUIRE(served != q.none);
    q.push(TempReading{101});        // posts arriving during the dispatch -
    q.push(TempReading{102});        // from an ISR or from the dispatch itself
    q.push(TempReading{103});
    CHECK(q.size() == 3);            // depth of them, all accepted
    CHECK(q.overflows() == 0);
    q.push(TempReading{104});        // every slot occupied: overflow
    CHECK(q.overflows() == 1);

    // The dispatched event is intact in its slot through all of it.
    REQUIRE(std::holds_alternative<ButtonDown>(q.at(served)));
    CHECK(std::get<ButtonDown>(q.at(served)).id == 7);
    q.release(served);

    CHECK(std::get<TempReading>(*q.pop()).centi == 101);
    CHECK(std::get<TempReading>(*q.pop()).centi == 102);
    CHECK(std::get<TempReading>(*q.pop()).centi == 103);
    CHECK(q.take() == q.none);
}

TEST_CASE("the overflow moment is one rule: every slot occupied, waiting or outstanding") {
    HostPlatform::reset();
    EventQueue<uint8_t, 2, HostPlatform> q;   // three slots

    // Idle, nothing held: the spare slot takes a waiting event too.
    CHECK(fill(q, uint8_t{1}) == 3);
    CHECK(q.size() == 3);
    CHECK(q.capacity() == 2);
    while (q.pop()) {}

    // A dispatch in progress: depth places for the producers.
    q.push(9);
    const uint8_t d = q.take();
    CHECK(fill(q, uint8_t{2}) == 2);
    q.release(d);
    while (q.pop()) {}

    // A hold outside any dispatch: depth places, the hold's own slot gone.
    q.push(9);
    const uint8_t h = q.take();
    const Held<uint8_t> kept = q.hold();
    CHECK(q.take_hold());
    CHECK(fill(q, uint8_t{3}) == 2);
    while (q.pop()) {}
    // ... and while a further event is dispatched beside it: depth - 1.
    q.push(9);
    const uint8_t d2 = q.take();
    CHECK(fill(q, uint8_t{4}) == 1);
    q.release(d2);
    while (q.pop()) {}
    q.release(kept);
    CHECK(kept.handle() == h);
    CHECK(fill(q, uint8_t{5}) == 3);  // every slot back
}

TEST_CASE("the largest depth: 255 slots, 256 positions, the byte's own width") {
    HostPlatform::reset();
    EventQueue<uint16_t, 254, HostPlatform> q;
    for (uint16_t i = 0; i < 255; ++i) {
        q.push(i);
    }
    CHECK(q.size() == 255);
    q.push(999);
    CHECK(q.overflows() == 1);
    for (uint16_t round = 0; round < 3; ++round) {
        for (uint16_t i = 0; i < 255; ++i) {
            const uint8_t h = q.take();
            REQUIRE(h != q.none);
            CHECK(q.at(h) == static_cast<uint16_t>(round * 255u + i));
            q.release(h);
            q.push(static_cast<uint16_t>((round + 1u) * 255u + i));
        }
    }
    CHECK(q.size() == 255);
    CHECK(q.overflows() == 1);
}

TEST_CASE("push builds each alternative in its slot, and a whole Event too") {
    HostPlatform::reset();
    EventQueue<Event, 4, HostPlatform> q;
    q.push(Tick{});
    q.push(ButtonDown{9});
    q.push(TempReading{-3});
    q.push(Event{ButtonDown{11}});
    CHECK(std::holds_alternative<Tick>(*q.pop()));
    CHECK(std::get<ButtonDown>(*q.pop()).id == 9);
    CHECK(std::get<TempReading>(*q.pop()).centi == -3);
    CHECK(std::get<ButtonDown>(*q.pop()).id == 11);
}

TEST_CASE("pop() is take() with a copy out and the slot released at once") {
    HostPlatform::reset();
    EventQueue<uint8_t, 1, HostPlatform> q;   // two slots
    q.push(4);
    q.push(5);
    const uint32_t before = HostPlatform::CriticalSection::entries;
    CHECK_FALSE(q.empty());
    CHECK(q.size() == 2);
    CHECK(q.pop().value() == 4);
    q.push(6);                        // the popped slot is free again
    CHECK(q.overflows() == 0);
    CHECK(q.pop().value() == 5);
    CHECK(q.pop().value() == 6);
    CHECK_FALSE(q.pop().has_value());
    CHECK(HostPlatform::CriticalSection::entries == before + 2);   // the push and overflows()
}

TEST_CASE("a static queue is all zero: nothing to construct, nothing in .data") {
    using Q = EventQueue<Event, 8, HostPlatform>;
    static_assert(std::is_trivially_destructible_v<Q>);
    // Constant-initialised: a constexpr object of the type exists, so a
    // static one needs no constructor at run time - and every byte of it
    // is zero, the encodings' purpose (the ring XOR its position, the
    // free run's end stored as the fence, each position one place ahead,
    // the slots numbered from 1).
    static constexpr Q zero{};
    (void)zero;
    alignas(Q) static const unsigned char zeros[sizeof(Q)] = {};
    static Q q;                      // static storage: padding zeroed too
    CHECK(std::equal(reinterpret_cast<const unsigned char*>(&q),
                     reinterpret_cast<const unsigned char*>(&q) + sizeof(Q), zeros));
}

// ---------------------------------------------------------------------------
// The hold: hold() inside a dispatch keeps the slot, take_hold() tells the
// kernel, release() gives it back in any order.
// ---------------------------------------------------------------------------

TEST_CASE("hold() hands out the slot being dispatched, and take_hold() reports it once") {
    HostPlatform::reset();
    EventQueue<uint8_t, 3, HostPlatform> q;
    q.push(7);
    const uint8_t h = q.take();
    const Held<uint8_t> kept = q.hold();
    REQUIRE(kept);
    CHECK(kept.handle() == h);
    CHECK(q.at(kept) == 7);
    CHECK(q.hold() == kept);          // a second hold in one dispatch: the same slot
    CHECK(q.take_hold());             // the kernel sees the hold ...
    CHECK_FALSE(q.take_hold());       // ... once
    q.release(kept);
}

TEST_CASE("hold() finds its slot after the same dispatch released older holds") {
    // hold() reads the number back from the position the take left, the
    // last of the outstanding run: releases inside the dispatch write at
    // the run's other end and never reach it, even with every slot
    // outstanding and every older hold given back first.
    HostPlatform::reset();
    EventQueue<uint8_t, 3, HostPlatform> q;   // four slots, five positions
    for (uint8_t round = 0; round < 9; ++round) {   // every rotation of the ring
        std::vector<Held<uint8_t>> older;
        for (uint8_t i = 0; i < 3; ++i) {
            q.push(static_cast<uint8_t>(10 * round + i));
            (void)q.take();
            older.push_back(q.hold());
            CHECK(q.take_hold());
        }
        q.push(static_cast<uint8_t>(10 * round + 9));
        const uint8_t h = q.take();             // the dispatch begins: every slot outstanding
        for (size_t i = older.size(); i-- > 0;) {
            q.release(older[i]);              // given back youngest first, inside it
        }
        const Held<uint8_t> kept = q.hold();
        CHECK(kept.handle() == h);
        CHECK(q.at(kept) == 10 * round + 9);
        CHECK(q.take_hold());
        q.release(kept);
        q.push(1);                            // and a turn to rotate the ring
        (void)q.pop();
    }
    CHECK(q.overflows() == 0);
}

TEST_CASE("an unheld dispatch is the kernel's to release") {
    HostPlatform::reset();
    EventQueue<uint8_t, 3, HostPlatform> q;
    q.push(1);
    const uint8_t h = q.take();
    CHECK_FALSE(q.take_hold());
    q.release(h);
    CHECK(fill(q, uint8_t{2}) == 4);
    CHECK_FALSE(Held<uint8_t>{});     // default-constructed: null
    CHECK(Held<uint8_t>{q.none} == Held<uint8_t>{});
}

namespace {

/// Take every waiting event, hold it, return the handles in take order.
template <typename Q>
std::vector<Held<uint16_t>> take_and_hold(Q& q) {
    std::vector<Held<uint16_t>> held;
    for (uint8_t h = q.take(); h != q.none; h = q.take()) {
        held.push_back(q.hold());
        CHECK(q.take_hold());
    }
    return held;
}

/// Every slot free: exactly slot_count pushes accepted, then refusal.
template <typename Q>
void check_all_free(Q& q, uint16_t slots) {
    const uint16_t before = q.overflows();
    CHECK(fill(q, uint16_t{0xBEEF}) == slots);
    CHECK(q.overflows() == before + 1u);
    while (q.pop()) {}
}

} // namespace

TEST_CASE("holds released in order, in reverse and out of order: each costs its slot") {
    HostPlatform::reset();
    using Q = EventQueue<uint16_t, 4, HostPlatform>;   // five slots
    const std::vector<std::vector<uint8_t>> orders = {
        {0, 1, 2, 3}, {3, 2, 1, 0}, {1, 3, 0, 2}, {2, 0, 3, 1},
    };
    for (const auto& order : orders) {
        Q q;
        for (uint16_t i = 0; i < 4; ++i) {
            q.push(static_cast<uint16_t>(100 + i));
        }
        std::vector<Held<uint16_t>> held = take_and_hold(q);
        REQUIRE(held.size() == 4);
        uint8_t released = 0;
        for (uint8_t k : order) {
            // While some are held, the rest of the slots carry traffic.
            for (uint16_t turn = 0; turn < 7; ++turn) {
                q.push(turn);
                CHECK(q.pop().value() == turn);
            }
            for (uint8_t j = 0; j < 4; ++j) {
                if (held[j]) {
                    CHECK(q.at(held[j]) == 100 + j);   // intact while held
                }
            }
            q.release(held[k]);
            held[k] = {};
            ++released;
            CHECK(fill(q, uint16_t{1}) == 1u + released);
            while (q.pop()) {}
        }
        CHECK(q.overflows() == 4);
        check_all_free(q, 5);
    }
}

TEST_CASE("one hold outliving many wraps of the ring costs exactly its slot") {
    HostPlatform::reset();
    EventQueue<uint16_t, 2, HostPlatform> q;   // three slots, four positions
    q.push(1);
    const uint8_t h = q.take();
    const Held<uint16_t> kept = q.hold();
    REQUIRE(q.take_hold());
    REQUIRE(h != q.none);
    for (uint32_t turn = 0; turn < 100000; ++turn) {
        const uint16_t a = static_cast<uint16_t>(turn);
        const uint16_t b = static_cast<uint16_t>(turn + 1u);
        q.push(a);
        q.push(b);                    // both slots beside the hold
        const uint8_t ha = q.take();
        const uint8_t hb = q.take();
        REQUIRE(ha != q.none);
        REQUIRE(hb != q.none);
        REQUIRE(q.at(ha) == a);
        REQUIRE(q.at(hb) == b);
        if ((turn & 1u) != 0u) {      // release order alternates
            q.release(ha);
            q.release(hb);
        } else {
            q.release(hb);
            q.release(ha);
        }
        REQUIRE(q.at(kept) == 1);
    }
    CHECK(q.overflows() == 0);
    CHECK(fill(q, uint16_t{2}) == 2);   // the hold's slot and nothing more is gone
    while (q.pop()) {}
    q.release(kept);
    check_all_free(q, 3);
}

TEST_CASE("a long hold costs its own slot and nothing more, at any duration") {
    // The capacity regression of the structure: the obvious ring with a
    // release cursor would lose a slot per event dispatched behind the
    // hold and refuse every push within a few dozen turns.
    HostPlatform::reset();
    EventQueue<uint16_t, 6, HostPlatform> q;   // seven slots
    q.push(500);
    const uint8_t h0 = q.take();
    const Held<uint16_t> oldest = q.hold();
    REQUIRE(q.take_hold());
    REQUIRE(h0 != q.none);
    q.push(501);
    (void)q.take();
    const Held<uint16_t> second = q.hold();
    REQUIRE(q.take_hold());
    uint16_t next = 0;
    for (uint32_t turn = 0; turn < 50000; ++turn) {
        const uint8_t burst = static_cast<uint8_t>(1u + turn % 4u);   // 1..4 waiting
        for (uint8_t i = 0; i < burst; ++i) {
            q.push(next++);
        }
        for (uint8_t i = 0; i < burst; ++i) {
            const uint8_t h = q.take();
            REQUIRE(h != q.none);
            q.release(h);
        }
        if (turn == 25000) {
            q.release(second);        // the younger hold ends first
        }
    }
    CHECK(q.overflows() == 0);
    CHECK(q.at(oldest) == 500);
    CHECK(fill(q, uint16_t{3}) == 6);   // seven slots, one held
    while (q.pop()) {}
    q.release(oldest);
    check_all_free(q, 7);
}

TEST_CASE("the documentation's example: four slots, holds of three lengths") {
    // docs/design/kernel.md section 5, step by step: the slot numbers
    // take() hands out are the ones the tables there show.
    HostPlatform::reset();
    EventQueue<uint8_t, 3, HostPlatform> q;    // four slots, numbered 1 to 4: S1..S4
    q.push(1);                                  // E1 ...
    CHECK(q.take() == 1);                       // ... in S1, held long
    const Held<uint8_t> e1 = q.hold();
    q.push(2);                                  // E2 in S2, held medium
    CHECK(q.take() == 2);
    const Held<uint8_t> e2 = q.hold();
    q.push(3);                                  // E3 in S3, held long
    CHECK(q.take() == 3);
    const Held<uint8_t> e3 = q.hold();
    q.release(e2);                              // E2 done before E1
    q.push(4);                                  // E4 takes S4 ...
    const uint8_t h4 = q.take();
    CHECK(h4 == 4);
    q.release(h4);                              // ... and is released at once
    q.push(5);                                  // E5 takes S2, while S1, S3 held
    const uint8_t h5 = q.take();
    CHECK(h5 == 2);
    CHECK(q.at(h5) == 5);
    CHECK(q.at(e1) == 1);
    CHECK(q.at(e3) == 3);
    q.release(h5);
    q.release(e3);                              // then E3 and E1, in any order
    q.release(e1);
    check_all_free(q, 4);
}

// ---------------------------------------------------------------------------
// The interleaving explorer
// ---------------------------------------------------------------------------

namespace {

struct Ev {
    uint16_t seq;
    uint8_t pad[6];
};

/// The schedule: at which consumer-side boundaries the "ISR" pushes.
struct Explorer {
    static inline uint32_t point = 0;            // boundaries hit so far
    static inline std::vector<uint32_t> inject;  // where the ISR pushes (sorted)
    static inline uint8_t burst = 1;             // pushes per injection
    static inline void (*isr_push)() = nullptr;
    static inline bool counting = false;         // dry run: count only
    static inline bool in_isr = false;

    static void at() {
        // An interrupt cannot land inside a masked window, nor nest.
        REQUIRE(HostPlatform::CriticalSection::depth == 0);
        if (!counting && !in_isr && isr_push != nullptr &&
            std::binary_search(inject.begin(), inject.end(), point)) {
            in_isr = true;
            for (uint8_t i = 0; i < burst; ++i) {
                isr_push();
            }
            in_isr = false;
        }
        ++point;
    }
};

/// The host platform with the optional interleave_point() the queue's
/// consumer verbs call.
struct ExplorerPlatform : HostPlatform {
    static void interleave_point() { Explorer::at(); }
};
static_assert(brio::Platform<ExplorerPlatform>);

constexpr uint8_t explored_depth = 4;            // five slots: not a power of two
constexpr uint16_t explored_slots = explored_depth + 1u;
using XQ = EventQueue<Ev, explored_depth, ExplorerPlatform>;

/// What the queue must do, kept beside it.
struct Model {
    XQ* q = nullptr;
    uint16_t seq = 1;
    std::deque<uint16_t> pending;          // accepted, not yet taken, in time order
    std::vector<uint8_t> out;              // taken, not released: slot numbers
    std::vector<uint16_t> out_seq;         // ... and the seq each must still hold
    std::vector<uint16_t> taken;           // every seq ever taken
    uint32_t accepted = 0;
    uint32_t dropped = 0;
    bool releasing = false;                // a release in progress

    void push_one() {
        const size_t occ = pending.size() + out.size();
        const uint16_t before = q->overflows();
        q->push(Ev{seq, {}});
        const bool refused = q->overflows() != before;
        // The documented moment: refused exactly when every slot is
        // occupied, waiting or outstanding. A release in progress (its
        // slot already gone from the model) takes effect at the queue's
        // fence store, so inside one the slot may still count.
        if (releasing) {
            REQUIRE((refused ? occ + 1u >= explored_slots : occ < explored_slots));
        } else {
            REQUIRE(refused == (occ == explored_slots));
        }
        if (refused) {
            ++dropped;
        } else {
            pending.push_back(seq);
            ++accepted;
        }
        ++seq;
    }

    void take() {
        const std::deque<uint16_t> at_start = pending;
        const uint8_t h = q->take();
        if (h == XQ::none) {
            // linearizable: empty at its start (an injection inside it may
            // have pushed after the emptiness was seen)
            REQUIRE(at_start.empty());
            return;
        }
        const uint16_t got = q->at(h).seq;
        // the oldest waiting at its start, or - empty at its start - the
        // oldest an injection inside it pushed
        const uint16_t expect = at_start.empty() ? pending.front() : at_start.front();
        REQUIRE(got == expect);
        REQUIRE(pending.front() == got);
        pending.pop_front();
        REQUIRE(std::find(out.begin(), out.end(), h) == out.end());   // never handed out twice
        out.push_back(h);
        out_seq.push_back(got);
        taken.push_back(got);
    }

    void release(size_t k) {   // the k-th outstanding, oldest first
        if (k >= out.size()) {
            return;
        }
        const uint8_t h = out[k];
        out.erase(out.begin() + static_cast<long>(k));
        out_seq.erase(out_seq.begin() + static_cast<long>(k));
        releasing = true;
        q->release(h);
        releasing = false;
    }

    /// An outstanding slot is nobody else's: its bytes are what was built.
    void check_outstanding_intact() const {
        for (size_t i = 0; i < out.size(); ++i) {
            REQUIRE(q->at(out[i]).seq == out_seq[i]);
        }
    }
};

/// A script: P = a push from the loop, T = take (the slot stays
/// outstanding), 0..9 = release the k-th outstanding (oldest first).
void run_script(Model& m, const std::string& script) {
    for (char c : script) {
        if (c == 'P') {
            m.push_one();
        } else if (c == 'T') {
            m.take();
        } else if (c >= '0' && c <= '9') {
            m.release(static_cast<size_t>(c - '0'));
        }
        m.check_outstanding_intact();
    }
}

/// Take and release until nothing moves: the injections land inside the
/// drain's own takes and releases too.
void drain(Model& m) {
    for (int guard = 0; guard < 1000; ++guard) {
        const size_t before = m.taken.size();
        m.take();
        if (m.taken.size() != before || !m.pending.empty()) {
            continue;
        }
        if (!m.out.empty()) {
            m.release(m.out.size() - 1);   // the youngest first: out of order
            continue;
        }
        break;
    }
    REQUIRE(m.pending.empty());
    REQUIRE(m.out.empty());
    REQUIRE(m.q->empty());
}

Model* current_model = nullptr;
void isr_push() { current_model->push_one(); }

/// One scenario under every single-point and every two-point schedule,
/// with bursts of one and two pushes. Returns the schedules run.
uint32_t explore(const std::string& script) {
    uint32_t runs = 0;
    for (const uint8_t burst : {uint8_t{1}, uint8_t{2}}) {
        auto one = [&](const std::vector<uint32_t>& sched, bool counting) {
            HostPlatform::reset();
            XQ q;
            Model m;
            m.q = &q;
            current_model = &m;
            Explorer::point = 0;
            Explorer::inject = sched;
            Explorer::counting = counting;
            Explorer::isr_push = &isr_push;
            Explorer::burst = burst;
            run_script(m, script);
            drain(m);
            const uint32_t points = Explorer::point;
            Explorer::counting = true;           // the checks below inject nothing
            // nothing lost, nothing duplicated
            std::vector<uint16_t> t = m.taken;
            std::sort(t.begin(), t.end());
            REQUIRE(std::adjacent_find(t.begin(), t.end()) == t.end());
            REQUIRE(t.size() == m.accepted);
            REQUIRE(m.accepted + m.dropped == static_cast<uint32_t>(m.seq - 1u));
            // and every slot free again: exactly slot-count pushes accepted
            const uint16_t before = q.overflows();
            uint16_t accepted = 0;
            for (uint16_t i = 0; i <= explored_slots; ++i) {
                const uint16_t o = q.overflows();
                q.push(Ev{0, {}});
                if (q.overflows() == o) {
                    ++accepted;
                }
            }
            REQUIRE(accepted == explored_slots);
            REQUIRE(q.overflows() == before + 1u);
            return points;
        };
        const uint32_t k_points = one({}, true);
        REQUIRE(k_points > 0);
        for (uint32_t k = 0; k < k_points; ++k) {
            (void)one({k}, false);
            ++runs;
        }
        for (uint32_t k1 = 0; k1 < k_points; ++k1) {
            for (uint32_t k2 = k1 + 1; k2 < k_points; ++k2) {
                (void)one({k1, k2}, false);
                ++runs;
            }
        }
    }
    return runs;
}

} // namespace

TEST_CASE("the explorer: every single and paired interleaving of the six scripts") {
    const std::vector<std::string> scripts = {
        "PTPTPTPTPTPTPTPT",                       // tight turnover
        "PPPTT0TPT",                              // a hold released in order
        "PPPPTTT1TPP0T0PT",                       // holds released OUT of order
        "PPPPPPPTTTTT43210PPPPPTTTTT",            // fill, overflow, drain, refill
        "PTTPTPT0PT0PT0PT0PT0PT0PT0PT0PT0PT0PT",  // one hold outliving many turns (wrap)
        "PPTT1PTPT1PT1PT1PT1PT1PT10PT",           // the oldest held across a dozen turns
    };
    uint32_t runs = 0;
    for (const auto& s : scripts) {
        runs += explore(s);
    }
    MESSAGE("interleavings explored: " << runs);
    CHECK(runs == 20054);   // the study's count for this structure: a lost point() shows
}

TEST_CASE("the explorer's consumer takes no critical section") {
    HostPlatform::reset();
    XQ q;
    Explorer::counting = true;
    q.push(Ev{1, {}});
    q.push(Ev{2, {}});
    CHECK(HostPlatform::CriticalSection::entries == 2);
    const uint8_t a = q.take();
    const uint8_t b = q.take();
    REQUIRE(a != XQ::none);
    REQUIRE(b != XQ::none);
    q.release(b);
    q.release(a);
    CHECK(q.take() == XQ::none);
    CHECK(HostPlatform::CriticalSection::entries == 2);
}
