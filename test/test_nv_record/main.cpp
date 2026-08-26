// Host tests for the nonvolatile-storage services: the record layout
// and its only-changed-bytes write policy (util/nv_record.hpp), the
// checksum they rest on (util/crc.hpp), the interrupt-paced writer AO
// (util/nv_writer.hpp) and the persistent panic record
// (util/persistent_panic.hpp).
//
// None of it knows what an EEPROM is: the store is a concept, and here
// it is an array with a counter on it. What runs on the silicon is the
// same source with avrdx/nvm.hpp's EepromStore in its place.
//
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>
#include <optional>

#include "host/platform_host.hpp"
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

    static void reset(bool deferred = false) {
        for (uint16_t i = 0; i < bytes; ++i) {
            cells[i] = 0xFF;
        }
        writes = refusals = finishes = 0;
        refuse_next = busy_flag = false;
        defer = deferred;
        interrupt_armed = ready_flag_ = false;
    }

    static constexpr uint16_t size() { return bytes; }
    static uint8_t read(uint16_t a) { return a < bytes ? cells[a] : 0xFF; }

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

/// Drain both queues the way Kernel::run would, standing in for the ISR
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
