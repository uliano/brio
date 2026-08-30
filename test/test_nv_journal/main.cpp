// Host tests for the small-value flash journal: util/nv_journal.hpp over
// brio/host/sim_flash.hpp.
//
// Nothing here knows what a SAM C21 is. The journal sees a FlashMedia and
// the media is RAM with counters on it, which is what makes the two
// things a real part cannot give testable at all: the SAME source is run
// over THREE flash shapes - the SAM C21's RWWEE array (256-byte erase
// unit, 64-byte program unit), the STM32G0's main array (2 KB and an
// ECC-guarded 8-byte double word, which is why the journal is being built
// before that target exists) and an AVR Dx-shaped one (512 and 2, where a
// single entry header spans six program units) - and a power loss is
// swept across EVERY program unit of a save and of a collection.
//
// Run with: ctest --preset host (or ctest --preset host -R test_nv_journal)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>
#include <optional>
#include <span>

#include "host/sim_flash.hpp"
#include "util/crc.hpp"
#include "util/nv_journal.hpp"

using namespace brio;

namespace {

/// The three shapes, each with the journal geometry that fits it. The
/// SAM's program unit is a quarter of its erase unit, so one half must be
/// two rows for six ids plus the reserve to fit at all - which is exactly
/// the arithmetic the static_assert in the header performs.
struct SamShape {
    using M = SimFlash<256, 64, 8u * 1024u, 1>;
    using J = NvJournal<M, 6, 32, 2>;
};
struct G0Shape {
    using M = SimFlash<2048, 8, 64u * 1024u, 1>;
    using J = NvJournal<M, 6, 32, 1>;
};
struct AvrShape {
    using M = SimFlash<512, 2, 128u * 1024u, 2>;
    using J = NvJournal<M, 6, 32, 1>;
};

static_assert(FlashMedia<SamShape::M>);
static_assert(FlashMedia<G0Shape::M>);
static_assert(FlashMedia<AvrShape::M>);

/// A virgin part whose whole extent is offered to the journal. The
/// journal carves its own region out of the top by itself, exactly as
/// the heap does with its map home.
template <typename S>
void fresh_media() {
    using M = typename S::M;
    M::reset();
    M::set_build_id(0xA1B2C3D4u);
    for (uint8_t i = 0; i < M::zone_count; ++i) {
        M::set_zone(i, M::flash_end, 0);
    }
}

uint8_t work[64];
uint8_t back[64];
uint8_t ref[64];

void fill(uint8_t* p, uint32_t n, uint8_t seed) {
    for (uint32_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + i * 7u + (i >> 3));
    }
}

bool same(const uint8_t* a, const uint8_t* b, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

template <typename J>
bool put(J& j, uint8_t id, uint8_t len, uint8_t seed) {
    fill(work, len, seed);
    return j.save(id, std::span<const uint8_t>(work, len));
}

template <typename J>
bool holds(const J& j, uint8_t id, uint8_t len, uint8_t seed) {
    const std::optional<uint8_t> n = j.load(id, std::span<uint8_t>(back, 64));
    if (!n || *n != len) {
        return false;
    }
    fill(ref, len, seed);
    return same(ref, back, len);
}

/// A deterministic stand-in for randomness: the same sequence every run,
/// because a test that fails only sometimes proves nothing.
uint32_t xorshift(uint32_t& s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

} // namespace

// ---------------------------------------------------------------------------
//  mounting
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("a virgin part mounts as an empty journal, and mounting "
                   "costs nothing", S, SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    J j;
    const auto& r = j.mount();
    CHECK(r.status == NvJournalStatus::empty);
    CHECK(r.live == 0);
    CHECK(r.torn == 0);
    CHECK(r.seq == 0);
    CHECK(r.used_cells == 0);
    CHECK(!r.collect_pending);
    CHECK(j.mounted());
    CHECK(!j.has(1));
    CHECK(!j.load(1, std::span<uint8_t>(back, 8)).has_value());
    // READ-ONLY: not one erase, not one program.
    CHECK(M::total_erases() == 0);
    CHECK(M::cells_programmed == 0);
    CHECK(M::double_programs == 0);
}

TEST_CASE_TEMPLATE("garbage in the journal home mounts as an empty journal", S,
                   SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    uint8_t noise[4096];
    for (uint32_t i = 0; i < 2u * J::half_bytes; ++i) {
        noise[i] = static_cast<uint8_t>(0x37u + i * 11u);
    }
    M::reflash(J::journal_home,
               std::span<const uint8_t>(noise, 2u * J::half_bytes));
    J j;
    const auto& r = j.mount();
    CHECK(r.status == NvJournalStatus::empty);
    CHECK(r.live == 0);
    // And it is usable from there: the collection that the dirty half
    // forces is the journal's own business.
    CHECK(put(j, 7, 20, 0x11));
    CHECK(holds(j, 7, 20, 0x11));
}

TEST_CASE_TEMPLATE("a floor inside the journal home REFUSES the mount", S,
                   SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    // The image grew: its read-only data now reaches into the region.
    M::set_zone(0, M::flash_end, J::journal_home + J::half_bytes);
    J j;
    const auto& r = j.mount();
    CHECK(r.status == NvJournalStatus::bad_geometry);
    CHECK(!j.mounted());
    CHECK(!put(j, 1, 8, 0x22));
    CHECK(j.last_error() == NvJournalError::not_mounted);
    CHECK(M::total_erases() == 0);
    CHECK(M::cells_programmed == 0);
}

TEST_CASE_TEMPLATE("an unmounted journal refuses everything", S, SamShape,
                   G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    J j;
    CHECK(!j.mounted());
    CHECK(!j.has(1));
    CHECK(!j.load(1, std::span<uint8_t>(back, 8)).has_value());
    CHECK(!put(j, 1, 8, 0x01));
    CHECK(!j.save_reserved(1, std::span<const uint8_t>(work, 8)));
    CHECK(!j.collect());
    CHECK(M::total_erases() == 0);
    CHECK(M::cells_programmed == 0);
}

// ---------------------------------------------------------------------------
//  the round trip
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("a value is saved, found, read back and survives a remount",
                   S, SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    {
        J j;
        REQUIRE(j.mount().status == NvJournalStatus::empty);
        REQUIRE(put(j, 3, 17, 0x5A));
        CHECK(j.count() == 1);
        CHECK(j.sequence() == 1);
        CHECK(holds(j, 3, 17, 0x5A));
        CHECK(j.has(3));
        // A destination too small is refused, never truncated.
        CHECK(!j.load(3, std::span<uint8_t>(back, 16)).has_value());
    }
    J again;
    const auto& r = again.mount();
    CHECK(r.status == NvJournalStatus::ok);
    CHECK(r.live == 1);
    CHECK(r.torn == 0);
    CHECK(r.seq == 1);
    CHECK(holds(again, 3, 17, 0x5A));
    CHECK(M::double_programs == 0);
    CHECK(M::misaligned == 0);
}

TEST_CASE_TEMPLATE("the typed verbs carry a struct both ways", S, SamShape,
                   G0Shape, AvrShape) {
    using J = typename S::J;
    struct Cal {
        int32_t offset;
        uint16_t gain;
        uint8_t mode;
    };
    fresh_media<S>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    REQUIRE(j.template save<Cal>(2, Cal{-1234, 4001, 7}));
    const std::optional<Cal> got = j.template load<Cal>(2);
    REQUIRE(got.has_value());
    CHECK(got->offset == -1234);
    CHECK(got->gain == 4001);
    CHECK(got->mode == 7);
    // A different type of a different size reads as absent, not as
    // garbage: the stored length has to match exactly.
    CHECK(!j.template load<uint32_t>(2).has_value());
    CHECK(!j.template load<Cal>(3).has_value());
}

TEST_CASE_TEMPLATE("the latest save of an id is the one that rules", S,
                   SamShape, G0Shape, AvrShape) {
    using J = typename S::J;
    fresh_media<S>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    for (uint8_t k = 1; k <= 5; ++k) {
        REQUIRE(put(j, 4, static_cast<uint8_t>(k * 3), k));
        CHECK(j.count() == 1);        // one id, always
        CHECK(holds(j, 4, static_cast<uint8_t>(k * 3), k));
    }
    J again;
    CHECK(again.mount().live == 1);
    CHECK(holds(again, 4, 15, 5));
}

TEST_CASE_TEMPLATE("a zero-length value is a value", S, SamShape, G0Shape,
                   AvrShape) {
    using J = typename S::J;
    fresh_media<S>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    REQUIRE(j.save(9, std::span<const uint8_t>()));
    CHECK(j.has(9));
    const std::optional<uint8_t> n = j.load(9, std::span<uint8_t>(back, 8));
    REQUIRE(n.has_value());
    CHECK(*n == 0);
    J again;
    CHECK(again.mount().live == 1);
    CHECK(again.has(9));
}

// ---------------------------------------------------------------------------
//  the entry layout, pinned
// ---------------------------------------------------------------------------

TEST_CASE("the entry layout is exactly what the header documents") {
    using M = SamShape::M;
    using J = SamShape::J;
    fresh_media<SamShape>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    const uint8_t payload[3] = {0xAA, 0xBB, 0xCC};
    REQUIRE(j.save(0x11, std::span<const uint8_t>(payload, 3)));

    uint8_t cell[64];
    M::read(J::half_base(0), std::span<uint8_t>(cell, 64));
    // magic 'N','J' little-endian, id, length, seq little-endian,
    // CRC-16/CCITT-FALSE over the first eight header bytes then the
    // payload, a reserved 0xFFFF, the payload, then erased padding.
    CHECK(cell[0] == 0x4E);
    CHECK(cell[1] == 0x4A);
    CHECK(cell[2] == 0x11);
    CHECK(cell[3] == 0x03);
    CHECK(cell[4] == 0x01);
    CHECK(cell[5] == 0x00);
    CHECK(cell[6] == 0x00);
    CHECK(cell[7] == 0x00);
    CHECK(cell[8] == 0xD8);
    CHECK(cell[9] == 0x4D);
    CHECK(cell[10] == 0xFF);
    CHECK(cell[11] == 0xFF);
    CHECK(cell[12] == 0xAA);
    CHECK(cell[13] == 0xBB);
    CHECK(cell[14] == 0xCC);
    for (uint32_t i = 15; i < 64; ++i) {
        CHECK(cell[i] == 0xFF);
    }
    // And the constants the layout is described by are the ones used.
    CHECK(J::header_bytes == 12);
    CHECK(J::magic == 0x4A4Eu);
    CHECK(J::journal_home == M::flash_end - 4u * 256u);
}

// ---------------------------------------------------------------------------
//  refusals
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("a payload larger than max_payload can never fit and is "
                   "refused", S, SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    const uint32_t erases = M::total_erases();
    const uint32_t cells = M::cells_programmed;
    CHECK(!j.save(1, std::span<const uint8_t>(work, 33)));
    CHECK(j.last_error() == NvJournalError::too_big);
    CHECK(!j.save_reserved(1, std::span<const uint8_t>(work, 33)));
    CHECK(j.last_error() == NvJournalError::too_big);
    CHECK(M::total_erases() == erases);
    CHECK(M::cells_programmed == cells);
}

TEST_CASE_TEMPLATE("max_ids is a declared limit and it is enforced", S,
                   SamShape, G0Shape, AvrShape) {
    using J = typename S::J;
    fresh_media<S>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    for (uint8_t id = 0; id < 6; ++id) {
        REQUIRE(put(j, id, 8, static_cast<uint8_t>(id + 1)));
    }
    CHECK(j.count() == 6);
    CHECK(!put(j, 6, 8, 0x99));
    CHECK(j.last_error() == NvJournalError::too_many_ids);
    // An id already live needs no new slot.
    CHECK(put(j, 3, 12, 0x55));
    for (uint8_t id = 0; id < 6; ++id) {
        if (id != 3) {
            CHECK(holds(j, id, 8, static_cast<uint8_t>(id + 1)));
        }
    }
    CHECK(holds(j, 3, 12, 0x55));
}

// ---------------------------------------------------------------------------
//  collection
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("the halves ping-pong: a collection moves every live value "
                   "and erases the half behind it", S, SamShape, G0Shape,
                   AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    for (uint8_t id = 0; id < 6; ++id) {
        REQUIRE(put(j, id, 32, static_cast<uint8_t>(id * 16 + 1)));
    }
    const uint8_t first = j.active_half();
    CHECK(M::total_erases() == 0);      // nothing has needed one yet

    // Churn one id until the half must be recycled, and watch it happen.
    uint32_t collections = 0;
    uint8_t here = first;
    for (uint32_t k = 0; k < 200; ++k) {
        REQUIRE(put(j, 0, 32, static_cast<uint8_t>(k)));
        if (j.active_half() != here) {
            ++collections;
            here = j.active_half();
        }
        REQUIRE(j.reserve_intact());
    }
    CHECK(collections > 2);
    // The halves alternate, and the wear is shared between them.
    const uint32_t low = M::erases_of_page(J::half_base(0) / M::erase_size);
    const uint32_t high = M::erases_of_page(J::half_base(1) / M::erase_size);
    CHECK(low > 0);
    CHECK(high > 0);
    CHECK((low > high ? low - high : high - low) <= 1);

    // Every value came through, and so did the churned one.
    for (uint8_t id = 1; id < 6; ++id) {
        CHECK(holds(j, id, 32, static_cast<uint8_t>(id * 16 + 1)));
    }
    CHECK(holds(j, 0, 32, 199));
    J again;
    const auto& r = again.mount();
    CHECK(r.status == NvJournalStatus::ok);
    CHECK(r.live == 6);
    CHECK(!r.collect_pending);
    CHECK(holds(again, 0, 32, 199));
    for (uint8_t id = 1; id < 6; ++id) {
        CHECK(holds(again, id, 32, static_cast<uint8_t>(id * 16 + 1)));
    }
    CHECK(M::double_programs == 0);
}

TEST_CASE_TEMPLATE("an explicit collection is a no-op the values do not "
                   "notice", S, SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    for (uint8_t id = 0; id < 4; ++id) {
        REQUIRE(put(j, id, 20, static_cast<uint8_t>(id + 1)));
    }
    const uint8_t before = j.active_half();
    REQUIRE(j.collect());
    CHECK(j.active_half() != before);
    CHECK(j.count() == 4);
    for (uint8_t id = 0; id < 4; ++id) {
        CHECK(holds(j, id, 20, static_cast<uint8_t>(id + 1)));
    }
    CHECK(M::double_programs == 0);
}

// ---------------------------------------------------------------------------
//  the panic reserve
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("after every completed save there is room for one more "
                   "maximum-size entry, and taking it costs no erase", S,
                   SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);

    uint32_t seed = 0x1234ABCDu;
    uint32_t reserved_saves = 0;
    for (uint32_t k = 0; k < 3000; ++k) {
        const uint8_t id = static_cast<uint8_t>(xorshift(seed) % 5u);
        const uint8_t len = static_cast<uint8_t>(xorshift(seed) % 33u);
        REQUIRE(put(j, id, len, static_cast<uint8_t>(k)));
        REQUIRE(j.reserve_intact());
        // The panic path, exercised right where a panic could happen:
        // a maximum-size entry, no collection, NO ERASE.
        fill(work, 32, static_cast<uint8_t>(k + 1));
        const uint32_t erases = M::total_erases();
        REQUIRE(j.save_reserved(5, std::span<const uint8_t>(work, 32)));
        REQUIRE(M::total_erases() == erases);
        REQUIRE(holds(j, 5, 32, static_cast<uint8_t>(k + 1)));
        ++reserved_saves;
    }
    CHECK(reserved_saves == 3000);
    CHECK(M::double_programs == 0);
    CHECK(M::misaligned == 0);

    // And it all still reads back after a remount.
    J again;
    CHECK(again.mount().status == NvJournalStatus::ok);
    CHECK(holds(again, 5, 32, static_cast<uint8_t>(2999 + 1)));
}

TEST_CASE_TEMPLATE("the panic reporter writes through the reserve and the boot "
                   "path takes it back", S, SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    static J j;
    using Panic = JournalPanic<j, 5>;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    REQUIRE(put(j, 0, 16, 0x41));
    CHECK(!Panic::pending());

    const uint32_t erases = M::total_erases();
    Panic::report(PanicCode::kernel_fault, 0x2A);
    CHECK(M::total_erases() == erases);   // the panic path never erases
    CHECK(Panic::pending());

    // A reboot: the record is still there and is handed over exactly
    // once, and the value that was stored beside it is untouched.
    {
        J boot;
        REQUIRE(boot.mount().status == NvJournalStatus::ok);
        using BootPanic = JournalPanic<j, 5>;
        (void)sizeof(BootPanic);
        CHECK(holds(boot, 0, 16, 0x41));
    }
    const std::optional<PanicRecord> r = Panic::take();
    REQUIRE(r.has_value());
    CHECK(r->magic == panic_magic);
    CHECK(r->code == static_cast<uint8_t>(PanicCode::kernel_fault));
    CHECK(r->context == 0x2A);
    CHECK(!Panic::pending());
    CHECK(!Panic::take().has_value());
    CHECK(holds(j, 0, 16, 0x41));

    // The take() left the reserve ready for the next failure.
    CHECK(j.reserve_intact());
}

// ---------------------------------------------------------------------------
//  the power-cut sweeps
// ---------------------------------------------------------------------------

/// The whole point of the append-only shape: cut the power at EVERY
/// program unit of a save and the journal comes back holding either the
/// old value or the new one - never half of either, and never nothing.
TEST_CASE_TEMPLATE("a save is atomic at every single write", S, SamShape,
                   G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    uint32_t units = 0;
    {
        fresh_media<S>();
        J j;
        REQUIRE(j.mount().status == NvJournalStatus::empty);
        REQUIRE(put(j, 1, 12, 0xA0));
        const uint32_t before = M::cells_programmed;
        REQUIRE(put(j, 1, 30, 0xB0));
        units = M::cells_programmed - before;
    }
    REQUIRE(units >= 1);

    for (uint32_t cut = 0; cut <= units; ++cut) {
        fresh_media<S>();
        {
            J j;
            REQUIRE(j.mount().status == NvJournalStatus::empty);
            REQUIRE(put(j, 1, 12, 0xA0));
            M::cut_after(cut);
            (void)put(j, 1, 30, 0xB0);
        }
        M::power_on();
        J again;
        const auto& r = again.mount();
        INFO("cut after ", cut, " of ", units, " program units");
        REQUIRE(r.status == NvJournalStatus::ok);
        REQUIRE(r.live == 1);
        const bool old_one = holds(again, 1, 12, 0xA0);
        const bool new_one = holds(again, 1, 30, 0xB0);
        CHECK((old_one != new_one));
        CHECK((old_one || new_one));
        // A torn tail is stepped over in SILENCE - reported, not fatal.
        CHECK((r.torn <= 1));
        CHECK(M::double_programs == 0);
        // And the journal is usable straight afterwards, torn tail or not.
        CHECK(put(again, 2, 8, 0xC0));
        CHECK(holds(again, 2, 8, 0xC0));
    }
}

/// The same sweep across a COLLECTION, which is the mutation that moves
/// every live value and erases both halves in turn.
TEST_CASE_TEMPLATE("a collection loses nothing at any single write", S,
                   SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    auto load_all = [](J& j) {
        for (uint8_t id = 0; id < 5; ++id) {
            REQUIRE(put(j, id, 24, static_cast<uint8_t>(id * 10 + 1)));
        }
    };

    uint32_t units = 0;
    {
        fresh_media<S>();
        J j;
        REQUIRE(j.mount().status == NvJournalStatus::empty);
        load_all(j);
        const uint32_t before = M::cells_programmed;
        REQUIRE(j.collect());
        units = M::cells_programmed - before;
    }
    REQUIRE(units >= 5);

    for (uint32_t cut = 0; cut <= units; ++cut) {
        fresh_media<S>();
        {
            J j;
            REQUIRE(j.mount().status == NvJournalStatus::empty);
            load_all(j);
            M::cut_after(cut);
            (void)j.collect();
        }
        M::power_on();
        J again;
        const auto& r = again.mount();
        INFO("cut after ", cut, " of ", units, " program units of a collection");
        REQUIRE(r.status == NvJournalStatus::ok);
        REQUIRE(r.live == 5);
        for (uint8_t id = 0; id < 5; ++id) {
            REQUIRE(holds(again, id, 24, static_cast<uint8_t>(id * 10 + 1)));
        }
        // Whatever state the cut left, the next ordinary save finishes
        // the job and everything is still there afterwards.
        REQUIRE(put(again, 0, 16, 0x77));
        CHECK(!again.collect_pending());
        CHECK(holds(again, 0, 16, 0x77));
        for (uint8_t id = 1; id < 5; ++id) {
            CHECK(holds(again, id, 24, static_cast<uint8_t>(id * 10 + 1)));
        }
        CHECK(M::double_programs == 0);
    }
}

// ---------------------------------------------------------------------------
//  the states only a torn ERASE can leave
// ---------------------------------------------------------------------------

/// A half is one or more erase units, so the erase of a half is not one
/// hardware operation and a power loss can leave the DESTINATION of a
/// collection holding stale entries. The rule that resolves it - the
/// newer half is the destination - is checked here by building the state
/// by hand, because the simulated power switch only opens between
/// program units.
TEST_CASE("stale survivors of a half-erased destination are discarded, not "
          "resurrected") {
    using M = SamShape::M;
    using J = SamShape::J;
    fresh_media<SamShape>();
    uint8_t live_half = 0;
    {
        J j;
        REQUIRE(j.mount().status == NvJournalStatus::empty);
        for (uint8_t id = 0; id < 4; ++id) {
            REQUIRE(put(j, id, 24, static_cast<uint8_t>(id + 1)));
        }
        REQUIRE(j.collect());
        live_half = j.active_half();
        // One id gets a newer value, so its old copy is a plausible
        // thing to find lying in the other half.
        REQUIRE(put(j, 0, 24, 0x40));
    }
    const uint8_t stale_half = static_cast<uint8_t>(1u - live_half);

    // Forge what a torn erase of a collection's DESTINATION leaves: an
    // entry that verifies, in the idle half, with a sequence number
    // older than everything in the live one. On this shape an entry is
    // exactly one program unit, so one cell is the whole forgery.
    uint8_t forged[64];
    M::read(J::half_base(live_half), std::span<uint8_t>(forged, 64));
    forged[4] = 1;   // seq = 1, older than anything still in force
    forged[5] = 0;
    forged[6] = 0;
    forged[7] = 0;
    uint16_t crc = crc16(forged, 8);
    crc = crc16(forged + 12, forged[3], crc);
    forged[8] = static_cast<uint8_t>(crc);
    forged[9] = static_cast<uint8_t>(crc >> 8);
    REQUIRE(M::program(J::half_base(stale_half),
                       std::span<const uint8_t>(forged, 64)));

    J j;
    const auto& r = j.mount();
    CHECK(r.status == NvJournalStatus::ok);
    CHECK(r.active == live_half);   // the NEWER half is the destination
    CHECK(r.collect_pending);       // and there is a collection to finish
    CHECK(r.live == 4);
    CHECK(holds(j, 0, 24, 0x40));
    for (uint8_t id = 1; id < 4; ++id) {
        CHECK(holds(j, id, 24, static_cast<uint8_t>(id + 1)));
    }
    // Finishing it erases the stale half and changes no value.
    REQUIRE(j.collect());
    CHECK(!j.collect_pending());
    CHECK(holds(j, 0, 24, 0x40));
    for (uint8_t id = 1; id < 4; ++id) {
        CHECK(holds(j, id, 24, static_cast<uint8_t>(id + 1)));
    }
    J again;
    CHECK(again.mount().live == 4);
    CHECK(!again.report().collect_pending);
    CHECK(again.report().torn == 0);
    CHECK(holds(again, 0, 24, 0x40));
}

// ---------------------------------------------------------------------------
//  the bench conventions
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("a reflash of the image leaves the journal alone", S,
                   SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    {
        J j;
        REQUIRE(j.mount().status == NvJournalStatus::empty);
        REQUIRE(put(j, 1, 20, 0x11));
        REQUIRE(put(j, 2, 20, 0x22));
    }
    // A new image over the bottom of the part: the journal lives at the
    // top, anchored to the silicon, and is not part of any link.
    uint8_t image[2048];
    for (uint32_t i = 0; i < sizeof image; ++i) {
        image[i] = static_cast<uint8_t>(i * 5u);
    }
    M::reflash(0, std::span<const uint8_t>(image, sizeof image));
    J j;
    const auto& r = j.mount();
    CHECK(r.status == NvJournalStatus::ok);
    CHECK(r.live == 2);
    CHECK(holds(j, 1, 20, 0x11));
    CHECK(holds(j, 2, 20, 0x22));
}

TEST_CASE_TEMPLATE("a chip erase leaves an empty, usable journal", S, SamShape,
                   G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    {
        J j;
        REQUIRE(j.mount().status == NvJournalStatus::empty);
        REQUIRE(put(j, 1, 20, 0x11));
    }
    M::chip_erase();
    J j;
    const auto& r = j.mount();
    CHECK(r.status == NvJournalStatus::empty);
    CHECK(r.live == 0);
    CHECK(!j.has(1));
    CHECK(put(j, 1, 20, 0x11));
    CHECK(holds(j, 1, 20, 0x11));
}

// ---------------------------------------------------------------------------
//  the ECC discipline, once for the whole suite
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("no program unit is ever written twice between erases", S,
                   SamShape, G0Shape, AvrShape) {
    using M = typename S::M;
    using J = typename S::J;
    fresh_media<S>();
    J j;
    REQUIRE(j.mount().status == NvJournalStatus::empty);
    uint32_t seed = 0x9E3779B9u;
    for (uint32_t k = 0; k < 500; ++k) {
        const uint8_t id = static_cast<uint8_t>(xorshift(seed) % 6u);
        const uint8_t len = static_cast<uint8_t>(xorshift(seed) % 33u);
        REQUIRE(put(j, id, len, static_cast<uint8_t>(k)));
    }
    // Remounting in the middle of a life changes nothing either.
    J again;
    REQUIRE(again.mount().status == NvJournalStatus::ok);
    for (uint32_t k = 0; k < 100; ++k) {
        REQUIRE(put(again, static_cast<uint8_t>(k % 6u), 8,
                    static_cast<uint8_t>(k)));
    }
    CHECK(M::double_programs == 0);
    CHECK(M::misaligned == 0);
}
