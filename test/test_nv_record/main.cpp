// Host tests for the nonvolatile-storage services: the record layout
// and its only-changed-bytes write policy (util/nv_record.hpp), the
// checksum they rest on and the CRC-32 that lives beside it
// (util/crc.hpp), the interrupt-paced writer AO
// (util/nv_writer.hpp) and the persistent panic record
// (util/persistent_panic.hpp).
//
// None of it knows what an EEPROM is: the store is a concept, and here
// it is an array with a counter on it - one store that takes a byte
// (FakeStore) and a family that also takes a run (RunStore<unit>, the
// NvRunStore refinement), so every writer is proven on both of its
// paths. What runs on the silicon is the same source with
// avrxt/nvm.hpp's EepromStore in its place.
//
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>
#include <cstring>
#include <optional>
#include <utility>

#include "host/platform.hpp"
#include "kernel/panic.hpp"
#include "util/crc.hpp"
#include "util/nv_record.hpp"
#include "util/nv_writer.hpp"
#include "util/persistent_panic.hpp"

using namespace brio;

namespace {

/// A store that behaves like the silicon's: erased bytes read 0xFF, a
/// write completes only when the test says so, and every byte that
/// reaches the array is counted (endurance is what the tests are about).
struct FakeStore {
    static constexpr uint16_t bytes = 64;

    static inline uint8_t cells[bytes];
    static inline uint16_t writes = 0;       ///< bytes actually committed
    static inline uint16_t refusals = 0;
    static inline bool refuse_next = false;
    static inline bool busy_flag = false;    ///< set by write, cleared by settle
    static inline uint16_t pending_addr = 0;
    static inline uint8_t pending_value = 0;
    static inline bool defer = false;        ///< true = writes need settle()
    static inline bool interrupt_armed = false;
    static inline bool ready_flag_ = false;
    static inline uint16_t finishes = 0;
    static inline uint16_t reads_while_busy = 0;   ///< reads during a write

    static void reset(bool deferred = false) {
        for (uint16_t i = 0; i < bytes; ++i) {
            cells[i] = 0xFF;
        }
        writes = refusals = finishes = reads_while_busy = 0;
        refuse_next = busy_flag = false;
        defer = deferred;
        interrupt_armed = ready_flag_ = false;
    }

    static constexpr uint16_t size() { return bytes; }
    static uint8_t read(uint16_t a) {
        if (busy_flag) {
            ++reads_while_busy;
        }
        return a < bytes ? cells[a] : 0xFF;
    }

    static bool write(uint16_t a, uint8_t v) {
        if (a >= bytes || refuse_next) {
            refuse_next = false;
            ++refusals;
            return false;
        }
        if (defer) {
            busy_flag = true;
            pending_addr = a;
            pending_value = v;
        } else {
            cells[a] = v;
            ++writes;
        }
        return true;
    }

    /// Complete a deferred write, the way the hardware's ready interrupt
    /// would announce it.
    static void settle() {
        if (!busy_flag) {
            return;
        }
        cells[pending_addr] = pending_value;
        ++writes;
        busy_flag = false;
        ready_flag_ = true;
    }

    static bool ready() { return !busy_flag; }
    static bool wait_ready() {
        settle();
        return true;
    }
    static void finish() { ++finishes; }

    static void arm_ready_interrupt(bool on) { interrupt_armed = on; }
    static bool ready_flag() { return ready_flag_; }
    static void clear_ready_flag() { ready_flag_ = false; }
};

static_assert(NvStore<FakeStore>);
static_assert(NvPacedStore<FakeStore>);

struct Settings {
    uint16_t a;
    uint8_t b;
    uint8_t c;
};
static_assert(sizeof(Settings) == 4);

using Rec = NvRecord<Settings, FakeStore, 8>;

} // namespace

TEST_CASE("crc16 is CCITT-FALSE and notices every single-byte change") {
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(crc16(check, sizeof check) == 0x29B1u);   // the standard's check value
    CHECK(crc16(nullptr, 0) == 0xFFFFu);

    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    const uint16_t base = crc16(data, sizeof data);
    for (uint8_t i = 0; i < sizeof data; ++i) {
        data[i] = static_cast<uint8_t>(data[i] ^ 0x01u);
        CHECK(crc16(data, sizeof data) != base);
        data[i] = static_cast<uint8_t>(data[i] ^ 0x01u);
    }
    CHECK(crc16(data, sizeof data) == base);
}

TEST_CASE("crc32_ethernet is CRC-32/MPEG-2, and one word is its four bytes") {
    // THE PUBLISHED CHECK VALUE. Every catalogue gives CRC-32/MPEG-2 -
    // polynomial 0x04C11DB7, initial value 0xFFFFFFFF, no reflection, no
    // final XOR - a check value of 0x0376E6E7 over the nine ASCII bytes
    // "123456789". That is what pins this model to something outside the
    // repository; a silicon block with the polynomial wired in is then
    // pinned to the model by its own bench suite.
    static constexpr uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    static_assert(crc32_ethernet_poly == 0x04C11DB7u);
    static_assert(crc32_ethernet_bytes(check, sizeof check) == 0x0376E6E7u);
    CHECK(crc32_ethernet_bytes(check, sizeof check) == 0x0376E6E7u);

    // An empty message is the initial value: nothing fed, nothing
    // changed - which is what makes reset-then-read a meaningful answer.
    CHECK(crc32_ethernet(nullptr, 0) == crc32_ethernet_init);
    CHECK(crc32_ethernet_bytes(nullptr, 0) == crc32_ethernet_init);

    // A WORD IS FOUR BYTES, MOST SIGNIFICANT FIRST: the property that
    // lets a program checksum a byte stream on hardware that takes only
    // words, and the reason the word form can be trusted at all.
    static constexpr uint32_t words[] = {0x31323334u, 0x35363738u};
    CHECK(crc32_ethernet(words, 2) == crc32_ethernet_bytes(check, 8));

    // Taken in pieces, the running value continues: two bytes fed one at
    // a time from the initial value equal the pair fed together.
    uint32_t running = crc32_ethernet_init;
    running = crc32_ethernet_byte(running, '1');
    running = crc32_ethernet_byte(running, '2');
    CHECK(running == crc32_ethernet_bytes(check, 2));

    // Every single-byte change is noticed.
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    const uint32_t base32 = crc32_ethernet_bytes(data, sizeof data);
    for (uint8_t i = 0; i < sizeof data; ++i) {
        data[i] = static_cast<uint8_t>(data[i] ^ 0x01u);
        CHECK(crc32_ethernet_bytes(data, sizeof data) != base32);
        data[i] = static_cast<uint8_t>(data[i] ^ 0x01u);
    }
    CHECK(crc32_ethernet_bytes(data, sizeof data) == base32);
}

TEST_CASE("an erased store holds no record") {
    FakeStore::reset();
    CHECK(!Rec::load().has_value());
    CHECK(!Rec::valid());
}

TEST_CASE("a stored value comes back, and only the record's bytes moved") {
    FakeStore::reset();
    const Settings s{0x1234, 0x56, 0x78};
    const uint16_t written = Rec::store(s);
    CHECK(written == nv_record_size<Settings>);   // erased store: every byte differs
    CHECK(FakeStore::writes == written);

    const std::optional<Settings> back = Rec::load();
    REQUIRE(back.has_value());
    CHECK(back->a == s.a);
    CHECK(back->b == s.b);
    CHECK(back->c == s.c);

    // Nothing outside [8, 8 + size) was touched.
    for (uint16_t i = 0; i < FakeStore::bytes; ++i) {
        if (i < Rec::begin || i >= Rec::end) {
            CHECK(FakeStore::cells[i] == 0xFF);
        }
    }
}

TEST_CASE("storing the same value again writes nothing at all") {
    FakeStore::reset();
    const Settings s{0x1234, 0x56, 0x78};
    (void)Rec::store(s);
    FakeStore::writes = 0;
    CHECK(Rec::store(s) == 0);
    CHECK(FakeStore::writes == 0);
    CHECK(Rec::valid());
}

TEST_CASE("changing one field costs that field and the checksum only") {
    FakeStore::reset();
    (void)Rec::store(Settings{0x1234, 0x56, 0x78});
    FakeStore::writes = 0;

    // c alone changes: one payload byte, plus however many of the two
    // CRC bytes differ. Never the magic, never the version.
    const uint16_t n = Rec::store(Settings{0x1234, 0x56, 0x79});
    CHECK(n >= 1);
    CHECK(n <= 3);
    CHECK(FakeStore::cells[Rec::begin] == nv_record_magic);
    CHECK(FakeStore::cells[Rec::begin + 1] == 1);
    const std::optional<Settings> back = Rec::load();
    REQUIRE(back.has_value());
    CHECK(back->c == 0x79);
}

TEST_CASE("a corrupted payload byte invalidates the record") {
    FakeStore::reset();
    (void)Rec::store(Settings{0x1234, 0x56, 0x78});
    REQUIRE(Rec::valid());
    FakeStore::cells[Rec::begin + 4] ^= 0x01u;
    CHECK(!Rec::load().has_value());
}

TEST_CASE("a record of another version is not read as this one") {
    FakeStore::reset();
    using V2 = NvRecord<Settings, FakeStore, 8, 2>;
    (void)V2::store(Settings{1, 2, 3});
    CHECK(V2::valid());
    CHECK(!Rec::valid());          // same bytes, other version byte
}

TEST_CASE("clear() invalidates with one byte and leaves the payload") {
    FakeStore::reset();
    (void)Rec::store(Settings{0x1234, 0x56, 0x78});
    FakeStore::writes = 0;
    CHECK(Rec::clear());
    CHECK(FakeStore::writes == 1);
    CHECK(!Rec::valid());
    CHECK(FakeStore::cells[Rec::begin + 4] == 0x34);   // payload untouched
    // Already clear: a second call writes nothing.
    FakeStore::writes = 0;
    CHECK(Rec::clear());
    CHECK(FakeStore::writes == 0);
}

TEST_CASE("a record that does not fit the store is refused, not truncated") {
    FakeStore::reset();
    using TooHigh = NvRecord<Settings, FakeStore, FakeStore::bytes - 4>;
    CHECK(TooHigh::store(Settings{1, 2, 3}) == 0);
    CHECK(!TooHigh::load().has_value());
}

// ---------------------------------------------------------------------------
//  the persistent panic record
// ---------------------------------------------------------------------------

TEST_CASE("a panic record survives in the store and is taken once") {
    FakeStore::reset();
    using Panic = PersistentPanic<FakeStore, 0>;
    CHECK(!Panic::pending());
    CHECK(!Panic::take().has_value());

    Panic::report(PanicCode::assert_failed, 0x5A);
    CHECK(Panic::pending());
    const std::optional<PanicRecord> r = Panic::take();
    REQUIRE(r.has_value());
    CHECK(r->magic == panic_magic);
    CHECK(r->code == static_cast<uint8_t>(PanicCode::assert_failed));
    CHECK(r->context == 0x5A);
    CHECK(!Panic::take().has_value());
    CHECK(!Panic::pending());
}

TEST_CASE("panic() through the persistent reporter leaves both breadcrumbs") {
    FakeStore::reset();
    HostPlatform::reset();
    using Panic = PersistentPanic<FakeStore, 0>;
    // panic() never returns, so the reporter is exercised the way the
    // kernel calls it plus the record write the kernel does first.
    HostPlatform::panic_record() =
        PanicRecord{panic_magic, static_cast<uint8_t>(PanicCode::kernel_fault), 9};
    Panic::report(PanicCode::kernel_fault, 9);

    const std::optional<PanicRecord> ram = take_panic_record<HostPlatform>();
    REQUIRE(ram.has_value());
    const std::optional<PanicRecord> nv = Panic::take();
    REQUIRE(nv.has_value());
    CHECK(nv->code == ram->code);
    CHECK(nv->context == ram->context);
}

TEST_CASE("save() round-trips a record the platform found in RAM") {
    FakeStore::reset();
    using Panic = PersistentPanic<FakeStore, 0>;
    CHECK(Panic::save(PanicRecord{panic_magic, 3, 0xAB}));
    const std::optional<PanicRecord> r = Panic::peek();
    REQUIRE(r.has_value());
    CHECK(r->context == 0xAB);
    CHECK(Panic::peek().has_value());        // peek does not consume
}

// ---------------------------------------------------------------------------
//  the writer AO
// ---------------------------------------------------------------------------

namespace {

using P = HostPlatform;
using Writer = NvWriter<FakeStore, P, 2>;

/// The AO's own reply mailbox: an AO whose only event is the reply.
struct Requester {
    using Event = NvDone;
    static inline EventQueue<Event, 8, P> queue;
    static inline uint8_t replies = 0;
    static inline NvDone last{};
    static void init() { replies = 0; last = NvDone{}; }
    static void dispatch(const Event& e) {
        ++replies;
        last = e;
    }
};

/// Drain both queues the way Tenuto::run would, standing in for the ISR
/// by settling the store and posting NvReady whenever a write is in
/// flight. Returns the number of dispatches, so a test can prove the
/// transfer really was spread over many of them.
uint16_t pump() {
    uint16_t steps = 0;
    for (uint16_t guard = 0; guard < 1000; ++guard) {
        if (const std::optional<Writer::Event> e = Writer::queue.pop()) {
            Writer::dispatch(*e);
            ++steps;
            continue;
        }
        if (const std::optional<Requester::Event> r = Requester::queue.pop()) {
            Requester::dispatch(*r);
            ++steps;
            continue;
        }
        if (FakeStore::interrupt_armed && !FakeStore::ready()) {
            FakeStore::settle();                    // the memory finishes
            FakeStore::arm_ready_interrupt(false);  // what the ISR must do
            post<Writer>(NvReady{});
            continue;
        }
        break;
    }
    return steps;
}

} // namespace

TEST_CASE("the writer commits a run of bytes one dispatch at a time") {
    FakeStore::reset(true);
    Requester::init();
    Writer::init();

    static const uint8_t payload[5] = {1, 2, 3, 4, 5};
    post<Writer>(NvWrite{16, lend<Lease::reply>(payload), sizeof payload,
                         reply_to<Requester, NvDone>()});
    const uint16_t steps = pump();

    CHECK(Requester::replies == 1);
    CHECK(Requester::last.status == nv_ok);
    CHECK(Requester::last.written == 5);
    for (uint8_t i = 0; i < 5; ++i) {
        CHECK(FakeStore::cells[16 + i] == payload[i]);
    }
    // One dispatch per byte at the very least: the point of the AO is
    // that nothing waits inside a single dispatch.
    CHECK(steps >= 5);
    CHECK(!FakeStore::interrupt_armed);
    CHECK(FakeStore::finishes >= 1);
}

TEST_CASE("bytes that already match are skipped, and an unchanged run "
          "never touches the memory") {
    FakeStore::reset(true);
    Requester::init();
    Writer::init();
    static const uint8_t payload[4] = {9, 9, 9, 9};
    post<Writer>(NvWrite{0, lend<Lease::reply>(payload), sizeof payload,
                         reply_to<Requester, NvDone>()});
    (void)pump();
    CHECK(Requester::last.written == 4);

    FakeStore::writes = 0;
    Requester::init();
    post<Writer>(NvWrite{0, lend<Lease::reply>(payload), sizeof payload,
                         reply_to<Requester, NvDone>()});
    (void)pump();
    CHECK(Requester::replies == 1);
    CHECK(Requester::last.status == nv_ok);
    CHECK(Requester::last.written == 0);
    CHECK(FakeStore::writes == 0);
}

TEST_CASE("a request that leaves the store is answered, not attempted") {
    FakeStore::reset(true);
    Requester::init();
    Writer::init();
    static const uint8_t payload[4] = {1, 2, 3, 4};
    post<Writer>(NvWrite{FakeStore::bytes - 2, lend<Lease::reply>(payload), sizeof payload,
                         reply_to<Requester, NvDone>()});
    (void)pump();
    CHECK(Requester::replies == 1);
    CHECK(Requester::last.status == nv_bad_range);
    CHECK(FakeStore::writes == 0);
}

TEST_CASE("requests queue behind the one in flight, and a full FIFO "
          "rejects instead of blocking") {
    FakeStore::reset(true);
    Requester::init();
    Writer::init();
    static const uint8_t a[3] = {1, 2, 3};
    static const uint8_t b[3] = {4, 5, 6};
    static const uint8_t c[3] = {7, 8, 9};
    static const uint8_t d[3] = {10, 11, 12};

    // All four arrive before any of them can finish: one goes in
    // flight, two wait in the FIFO (depth 2), the fourth is rejected.
    post<Writer>(NvWrite{0, lend<Lease::reply>(a), 3, reply_to<Requester, NvDone>()});
    Writer::dispatch(*Writer::queue.pop());        // starts the first
    post<Writer>(NvWrite{4, lend<Lease::reply>(b), 3, reply_to<Requester, NvDone>()});
    post<Writer>(NvWrite{8, lend<Lease::reply>(c), 3, reply_to<Requester, NvDone>()});
    post<Writer>(NvWrite{12, lend<Lease::reply>(d), 3, reply_to<Requester, NvDone>()});
    (void)pump();

    CHECK(Requester::replies == 4);
    CHECK(Writer::rejected_count() == 1);
    for (uint8_t i = 0; i < 3; ++i) {
        CHECK(FakeStore::cells[i] == a[i]);
        CHECK(FakeStore::cells[4 + i] == b[i]);
        CHECK(FakeStore::cells[8 + i] == c[i]);
        CHECK(FakeStore::cells[12 + i] == 0xFF);   // the rejected one
    }
}

TEST_CASE("a store that refuses a byte ends the request with nv_refused") {
    FakeStore::reset(true);
    Requester::init();
    Writer::init();
    static const uint8_t payload[4] = {1, 2, 3, 4};
    FakeStore::refuse_next = true;
    post<Writer>(NvWrite{0, lend<Lease::reply>(payload), sizeof payload,
                         reply_to<Requester, NvDone>()});
    (void)pump();
    CHECK(Requester::replies == 1);
    CHECK(Requester::last.status == nv_refused);
    CHECK(FakeStore::refusals == 1);
}

// ---------------------------------------------------------------------------
//  the run: a store that writes a run of one write unit in one operation
// ---------------------------------------------------------------------------

namespace {

/// A store with the contract's run verb, its write unit `unit` bytes
/// (the last unit cut short by the store's end when `unit` does not
/// divide it). It behaves as a page-buffered EEPROM does: an operation
/// touches only the bytes that differ, a read while an operation is in
/// flight waits it out (and is counted - the bus wait that holds a CPU's
/// interrupts on the AVR EA), and the power can be cut after any number
/// of operations, after which nothing reaches the array.
template <uint16_t unit>
struct RunStore {
    static constexpr uint16_t bytes = 64;

    static inline uint8_t cells[bytes];
    static inline uint16_t wear[bytes];     ///< operations that changed each byte
    static inline uint16_t ops = 0;         ///< operations started (runs and bytes)
    static inline uint16_t run_ops = 0;     ///< of which runs
    static inline uint16_t over_room = 0;   ///< runs refused for leaving their room
    static inline uint16_t reads_while_busy = 0;
    static inline uint16_t finishes = 0;
    static inline bool refuse_next = false;
    static inline int32_t power_left = -1;  ///< operations before the cut; -1 never
    static inline bool defer = false;
    static inline bool busy_flag = false;
    static inline bool interrupt_armed = false;
    static inline bool ready_flag_ = false;
    static inline uint16_t pending_at = 0;
    static inline uint16_t pending_len = 0;
    static inline uint8_t pending[bytes];

    static void reset(bool deferred = false) {
        for (uint16_t i = 0; i < bytes; ++i) {
            cells[i] = 0xFF;
            wear[i] = 0;
        }
        ops = run_ops = over_room = reads_while_busy = finishes = 0;
        refuse_next = busy_flag = interrupt_armed = ready_flag_ = false;
        power_left = -1;
        defer = deferred;
    }

    static constexpr uint16_t size() { return bytes; }
    static constexpr uint16_t run_room(uint16_t a) {
        const uint16_t to_unit = static_cast<uint16_t>(unit - a % unit);
        const uint16_t to_end = a < bytes ? static_cast<uint16_t>(bytes - a) : 1u;
        return to_unit < to_end ? to_unit : to_end;
    }

    static uint8_t read(uint16_t a) {
        if (busy_flag) {
            ++reads_while_busy;
            settle();
        }
        return a < bytes ? cells[a] : 0xFF;
    }

    static bool write_run(uint16_t a, const uint8_t* src, uint16_t n) {
        if (n == 0 || a >= bytes || n > run_room(a)) {
            ++over_room;
            return false;
        }
        if (refuse_next || busy_flag) {
            refuse_next = false;
            return false;
        }
        ++ops;
        ++run_ops;
        if (power_left == 0) {
            return true;              // the chip is off: nothing lands
        }
        if (power_left > 0) {
            --power_left;
        }
        pending_at = a;
        pending_len = n;
        for (uint16_t i = 0; i < n; ++i) {
            pending[i] = src[i];
        }
        busy_flag = true;
        if (!defer) {
            settle();
        }
        return true;
    }
    static bool write(uint16_t a, uint8_t v) {
        if (a >= bytes) {
            return false;
        }
        const bool ok = write_run(a, &v, 1);
        if (ok) {
            --run_ops;
        }
        return ok;
    }

    static void settle() {
        if (!busy_flag) {
            return;
        }
        for (uint16_t i = 0; i < pending_len; ++i) {
            const uint16_t at = static_cast<uint16_t>(pending_at + i);
            if (cells[at] != pending[i]) {    // byte marking: equal bytes untouched
                cells[at] = pending[i];
                ++wear[at];
            }
        }
        busy_flag = false;
        ready_flag_ = true;
    }

    static bool ready() { return !busy_flag; }
    static bool wait_ready() {
        settle();
        return true;
    }
    static void finish() { ++finishes; }

    static void arm_ready_interrupt(bool on) { interrupt_armed = on; }
    static bool ready_flag() { return ready_flag_; }
    static void clear_ready_flag() { ready_flag_ = false; }

    /// Write units in [a, a + n) holding a byte that differs from `want`.
    static uint16_t units_differing(uint16_t a, const uint8_t* want, uint16_t n) {
        uint16_t units = 0;
        uint16_t last_unit = 0xFFFFu;
        for (uint16_t i = 0; i < n; ++i) {
            const uint16_t at = static_cast<uint16_t>(a + i);
            if (cells[at] != want[i] && at / unit != last_unit) {
                ++units;
                last_unit = static_cast<uint16_t>(at / unit);
            }
        }
        return units;
    }
};

static_assert(NvRunStore<RunStore<8>>);
static_assert(NvPacedStore<RunStore<8>>);
static_assert(!NvRunStore<FakeStore>);   // the byte store stays a byte store

/// A record that spans several write units of every size swept below.
struct Wide {
    uint8_t b[13];
};
static_assert(nv_record_size<Wide> == 17);

template <typename T>
T pattern(uint8_t seed) {
    T v{};
    uint8_t* const p = reinterpret_cast<uint8_t*>(&v);
    for (uint16_t i = 0; i < sizeof(T); ++i) {
        p[i] = static_cast<uint8_t>(seed * 37u + i * 11u + 1u);
    }
    return v;
}

/// The values a record goes through: from erased, a whole new value,
/// the same again, one byte changed, a value with 0xFF bytes in it, and
/// back - every kind of change the writers distinguish.
template <typename T>
T step_value(uint8_t step) {
    T v = pattern<T>(1);
    uint8_t* const p = reinterpret_cast<uint8_t*>(&v);
    switch (step) {
        case 0: return v;
        case 1: return v;                              // unchanged
        case 2: p[sizeof(T) - 1] ^= 0x40u; return v;   // one byte
        case 3: p[0] = 0xFFu; p[sizeof(T) / 2] = 0xFFu; return v;
        case 4: return pattern<T>(7);                  // everything
        default: return pattern<T>(1);
    }
}
constexpr uint8_t value_steps = 6;

/// One record at one offset, stored through the same value sequence on
/// the byte store and on a run store of write unit `unit`: the same
/// bytes in the array, the same count returned, every run inside its
/// room, an operation per unit touched, no read of a busy store.
template <typename T, uint16_t unit, uint16_t offset>
void same_as_byte_path() {
    using ByteRec = NvRecord<T, FakeStore, offset>;
    using Store = RunStore<unit>;
    using RunRec = NvRecord<T, Store, offset>;
    FakeStore::reset();
    Store::reset(true);   // deferred: a read before a wait would show
    for (uint8_t step = 0; step < value_steps; ++step) {
        const T v = step_value<T>(step);
        uint8_t image[nv_record_size<T>];
        RunRec::build(v, image);
        const uint16_t units = Store::units_differing(offset, image, nv_record_size<T>);
        const uint16_t ops_before = Store::run_ops;
        const uint16_t n_byte = ByteRec::store(v);
        const uint16_t n_run = RunRec::store(v);
        CAPTURE(unit);
        CAPTURE(offset);
        CAPTURE(step);
        CHECK(n_run == n_byte);
        CHECK(Store::run_ops - ops_before == units);
        bool same = true;
        for (uint16_t i = 0; i < FakeStore::bytes; ++i) {
            if (FakeStore::cells[i] != Store::cells[i]) same = false;
        }
        CHECK(same);
        const std::optional<T> back = RunRec::load();
        REQUIRE(back.has_value());
        CHECK(std::memcmp(&*back, &v, sizeof(T)) == 0);
    }
    CHECK(Store::over_room == 0);
    CHECK(Store::reads_while_busy == 0);
    CHECK(Store::ops == Store::run_ops);   // the record never fell back to a byte
}

template <typename T, uint16_t unit, uint16_t... offsets>
void sweep_offsets(std::integer_sequence<uint16_t, offsets...>) {
    (same_as_byte_path<T, unit, offsets>(), ...);
}

template <typename T, uint16_t unit>
void sweep() {
    // Every offset of two whole units of the widest unit swept, so the
    // record starts at every position a room boundary can take.
    sweep_offsets<T, unit>(std::make_integer_sequence<uint16_t, 17>{});
}

} // namespace

TEST_CASE("the record on a run store: the byte path's bytes and count, a run "
          "per write unit touched, at every offset against every unit") {
    sweep<Settings, 1>();
    sweep<Settings, 2>();
    sweep<Settings, 3>();
    sweep<Settings, 4>();
    sweep<Settings, 5>();
    sweep<Settings, 8>();
    sweep<Settings, 16>();
    sweep<Wide, 1>();
    sweep<Wide, 3>();
    sweep<Wide, 4>();
    sweep<Wide, 8>();
    sweep<Wide, 16>();
    sweep<Wide, 64>();
}

TEST_CASE("the byte path reads the next byte while the last one is still "
          "being written; the run path waits before every read") {
    FakeStore::reset(true);
    (void)Rec::store(Settings{0x1234, 0x56, 0x78});
    CHECK(FakeStore::reads_while_busy > 0);

    using Store = RunStore<8>;
    using RunRec = NvRecord<Settings, Store, 4>;   // straddles a unit boundary
    Store::reset(true);
    (void)RunRec::store(Settings{0x1234, 0x56, 0x78});
    (void)RunRec::store(Settings{0x4321, 0x65, 0x87});
    CHECK(Store::run_ops == 4);   // two units, twice
    CHECK(Store::reads_while_busy == 0);
}

TEST_CASE("a power cut between the runs of a store leaves the old value, the "
          "new one or none - never another") {
    using Store = RunStore<4>;
    using RunRec = NvRecord<Wide, Store, 2>;        // 17 bytes over six units
    const Wide old_value = pattern<Wide>(3);
    const Wide new_value = pattern<Wide>(9);
    Store::reset();
    (void)RunRec::store(old_value);
    const uint16_t full = Store::run_ops;
    uint8_t image[nv_record_size<Wide>];
    RunRec::build(new_value, image);
    const uint16_t needed = Store::units_differing(2, image, nv_record_size<Wide>);
    REQUIRE(needed >= 2);
    uint16_t saw_none = 0;
    for (uint16_t cut = 0; cut <= needed; ++cut) {
        Store::reset();
        (void)RunRec::store(old_value);
        Store::power_left = cut;
        (void)RunRec::store(new_value);
        Store::power_left = -1;                     // the power is back
        const std::optional<Wide> back = RunRec::load();
        CAPTURE(cut);
        if (!back) {
            ++saw_none;
        } else {
            const bool is_old = std::memcmp(&*back, &old_value, sizeof(Wide)) == 0;
            const bool is_new = std::memcmp(&*back, &new_value, sizeof(Wide)) == 0;
            CHECK((is_old || is_new));
            CHECK(is_old == (cut == 0));
            CHECK(is_new == (cut == needed));
        }
        // And the next store, with the power on, completes it.
        (void)RunRec::store(new_value);
        const std::optional<Wide> again = RunRec::load();
        REQUIRE(again.has_value());
        CHECK(std::memcmp(&*again, &new_value, sizeof(Wide)) == 0);
    }
    CHECK(full >= 2);
    CHECK(saw_none == needed - 1);   // every cut strictly inside the store
}

TEST_CASE("the panic record on a run store is one operation, and is taken "
          "once") {
    using Store = RunStore<8>;
    using Panic = PersistentPanic<Store, 0>;
    using BytePanic = PersistentPanic<FakeStore, 0>;
    Store::reset();
    FakeStore::reset();
    Panic::report(PanicCode::assert_failed, 0x5A);
    BytePanic::report(PanicCode::assert_failed, 0x5A);
    CHECK(Store::run_ops == 1);
    CHECK(FakeStore::writes == nv_record_size<PanicRecord>);   // eight, one each
    for (uint16_t i = 0; i < 8; ++i) {
        CHECK(Store::cells[i] == FakeStore::cells[i]);
    }
    const std::optional<PanicRecord> r = Panic::take();
    REQUIRE(r.has_value());
    CHECK(r->code == static_cast<uint8_t>(PanicCode::assert_failed));
    CHECK(r->context == 0x5A);
    CHECK(Store::ops == 2);           // the take's clear is a byte: one more
    CHECK(!Panic::take().has_value());

    // Off a unit boundary the eight bytes straddle two units: two runs.
    using Straddling = PersistentPanic<Store, 12>;
    Store::reset();
    Straddling::report(PanicCode::kernel_fault, 1);
    CHECK(Store::run_ops == 2);
    CHECK(Straddling::pending());
}

// ---------------------------------------------------------------------------
//  the writer AO on a run store
// ---------------------------------------------------------------------------

namespace {

using RStore = RunStore<8>;
using RunWriter = NvWriter<RStore, P, 2>;

/// pump() for the run writer: the store's operation completes, the ISR
/// disarms and posts, exactly as for the byte writer.
uint16_t pump_run() {
    uint16_t steps = 0;
    for (uint16_t guard = 0; guard < 1000; ++guard) {
        if (const std::optional<RunWriter::Event> e = RunWriter::queue.pop()) {
            RunWriter::dispatch(*e);
            ++steps;
            continue;
        }
        if (const std::optional<Requester::Event> r = Requester::queue.pop()) {
            Requester::dispatch(*r);
            ++steps;
            continue;
        }
        if (RStore::interrupt_armed && !RStore::ready()) {
            RStore::settle();
            RStore::arm_ready_interrupt(false);
            post<RunWriter>(NvReady{});
            continue;
        }
        break;
    }
    return steps;
}

} // namespace

TEST_CASE("the writer on a run store: one operation per write unit touched, "
          "the byte writer's bytes and count") {
    // 20 bytes from offset 5 over units of 8: [5..7] [8..15] [16..23] [24].
    static uint8_t payload[20];
    for (uint8_t i = 0; i < sizeof payload; ++i) {
        payload[i] = static_cast<uint8_t>(0x30u + i);
    }
    RStore::reset(true);
    FakeStore::reset(true);
    Requester::init();
    RunWriter::init();
    Writer::init();

    post<RunWriter>(NvWrite{5, lend<Lease::reply>(payload), sizeof payload,
                            reply_to<Requester, NvDone>()});
    (void)pump_run();
    CHECK(Requester::replies == 1);
    CHECK(Requester::last.status == nv_ok);
    CHECK(Requester::last.written == 20);
    CHECK(RStore::run_ops == 4);
    CHECK(RStore::over_room == 0);
    CHECK(RStore::reads_while_busy == 0);
    CHECK(!RStore::interrupt_armed);
    CHECK(RStore::finishes == 1);

    post<Writer>(NvWrite{5, lend<Lease::reply>(payload), sizeof payload,
                         reply_to<Requester, NvDone>()});
    (void)pump();
    CHECK(Requester::replies == 2);
    CHECK(Requester::last.written == 20);
    for (uint16_t i = 0; i < FakeStore::bytes; ++i) {
        CHECK(FakeStore::cells[i] == RStore::cells[i]);
    }

    // Two bytes changed in the middle of the third unit, one in the
    // first: two operations, three bytes counted, the rest untouched.
    payload[0] = 0x00;
    payload[12] = 0x01;
    payload[14] = 0x02;
    const uint16_t ops_before = RStore::run_ops;
    post<RunWriter>(NvWrite{5, lend<Lease::reply>(payload), sizeof payload,
                            reply_to<Requester, NvDone>()});
    (void)pump_run();
    CHECK(Requester::last.status == nv_ok);
    CHECK(Requester::last.written == 3);
    CHECK(RStore::run_ops - ops_before == 2);
    for (uint8_t i = 0; i < sizeof payload; ++i) {
        CHECK(RStore::cells[5 + i] == payload[i]);
    }

    // Unchanged: answered without an operation or an interrupt.
    const uint16_t ops_now = RStore::run_ops;
    post<RunWriter>(NvWrite{5, lend<Lease::reply>(payload), sizeof payload,
                            reply_to<Requester, NvDone>()});
    (void)pump_run();
    CHECK(Requester::last.written == 0);
    CHECK(RStore::run_ops == ops_now);
}

TEST_CASE("the run writer answers a bad range and a refusal as the byte "
          "writer does") {
    static const uint8_t payload[4] = {1, 2, 3, 4};
    RStore::reset(true);
    Requester::init();
    RunWriter::init();
    post<RunWriter>(NvWrite{RStore::bytes - 2, lend<Lease::reply>(payload), sizeof payload,
                            reply_to<Requester, NvDone>()});
    (void)pump_run();
    CHECK(Requester::last.status == nv_bad_range);
    CHECK(RStore::ops == 0);

    RStore::refuse_next = true;
    post<RunWriter>(NvWrite{0, lend<Lease::reply>(payload), sizeof payload,
                            reply_to<Requester, NvDone>()});
    (void)pump_run();
    CHECK(Requester::replies == 2);
    CHECK(Requester::last.status == nv_refused);
    CHECK(Requester::last.written == 0);
}
