// Host tests for the flash block allocator: util/nv_heap.hpp over
// brio/host/sim_flash.hpp.
//
// Nothing here knows what an AVR is. The allocator sees a FlashMedia and
// the media is RAM with counters on it, which is what makes the two
// things a real part cannot give testable at all: the SAME source is run
// over an AVR-shaped flash (512-byte erase unit, 2-byte program unit)
// and an STM32G0-shaped one (2 KB and 8 bytes), and a power loss is
// swept across EVERY program unit of a mutation to prove the heap comes
// back as either entirely the old thing or entirely the new one.
//
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>
#include <optional>
#include <span>

#include "host/sim_flash.hpp"
#include "util/nv_heap.hpp"

using namespace brio;

namespace {

/// The two shapes the allocator must be right on. The AVR one is the
/// bench part; the G0 one erases four times as much and programs four
/// times as wide, and its program units may never be written twice.
using AvrFlash = SimFlash<512, 2, 128u * 1024u, 2>;
using G0Flash = SimFlash<2048, 8, 64u * 1024u, 2>;

static_assert(FlashMedia<AvrFlash>);
static_assert(FlashMedia<G0Flash>);

/// A generous default heap, and a cramped one for the capacity tests.
template <typename M>
using Heap = NvHeap<M, 8, 2>;

/// Zones by PAGE INDEX, so a test can say where things are without
/// arithmetic. The heap only ever sees byte addresses.
template <typename M>
void zones_pages(uint16_t mid_floor, uint16_t mid_ceiling, uint16_t tail_floor,
                 uint16_t tail_ceiling) {
    M::set_zone(0, static_cast<uint32_t>(mid_ceiling) * M::erase_size,
                static_cast<uint32_t>(mid_floor) * M::erase_size);
    M::set_zone(1, static_cast<uint32_t>(tail_ceiling) * M::erase_size,
                static_cast<uint32_t>(tail_floor) * M::erase_size);
}

/// A virgin part with two roomy zones: the middle one in the lower half,
/// the tail one reaching up to the map home. Same shape on both
/// geometries, expressed in pages of whatever they are.
template <typename M>
void fresh_media() {
    M::reset();
    M::set_build_id(0xA1B2C3D4u);
    const uint16_t half = static_cast<uint16_t>(M::page_count / 2);
    zones_pages<M>(4, static_cast<uint16_t>(half - 4),
                   static_cast<uint16_t>(half + 4),
                   static_cast<uint16_t>(M::page_count));
}

/// The number of pages the map home occupies, for the wear assertions.
template <typename M>
uint32_t map_page_index(uint8_t i) {
    return M::page_count - 2u + i;
}

uint8_t work[8192];   ///< what a test writes
uint8_t back[8192];   ///< what it reads back
uint8_t ref[8192];    ///< the pattern holds() compares against, kept apart
                      ///< from work[] so that checking a block never
                      ///< disturbs a payload a test is still building

void fill(uint8_t* p, uint32_t n, uint8_t seed) {
    for (uint32_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + i * 7u + (i >> 5));
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

/// Write one block and seal it. Returns false if anything refused.
template <typename M>
bool put(Heap<M>& heap, uint16_t id, uint32_t len, uint8_t seed) {
    fill(work, len, seed);
    std::optional<typename Heap<M>::Writer> w = heap.alloc(id, len);
    if (!w) {
        return false;
    }
    if (!w->append(std::span<const uint8_t>(work, len))) {
        return false;
    }
    return w->seal();
}

/// Read a block back and compare it with the pattern it was written
/// from. Returns false when the block is missing, the wrong length, or
/// the wrong bytes.
template <typename M>
bool holds(Heap<M>& heap, uint16_t id, uint32_t len, uint8_t seed) {
    const std::optional<NvBlock<M>> b = heap.find(id);
    if (!b || b->length != len) {
        return false;
    }
    if (!b->read(0, std::span<uint8_t>(back, len))) {
        return false;
    }
    fill(ref, len, seed);
    return same(ref, back, len);
}

} // namespace

// ---------------------------------------------------------------------------
//  mounting
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("a virgin part mounts as an empty heap", M, AvrFlash,
                   G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    const auto& r = heap.mount();
    CHECK(r.status == NvHeapStatus::empty);
    CHECK(r.survivors == 0);
    CHECK(r.lost == 0);
    CHECK(r.seq == 0);
    CHECK(heap.mounted());
    CHECK(heap.count() == 0);
    CHECK(!heap.find(1).has_value());
    // Read-only: mounting a virgin part costs no cycle at all.
    CHECK(M::total_erases() == 0);
    CHECK(M::cells_programmed == 0);
    CHECK(M::double_programs == 0);
}

TEST_CASE_TEMPLATE("garbage in the map home mounts as an empty heap", M,
                   AvrFlash, G0Flash) {
    fresh_media<M>();
    // Both rotation pages full of plausible-looking noise.
    fill(work, 2u * M::erase_size, 0x37);
    M::reflash(Heap<M>::map_home, std::span<const uint8_t>(work, 2u * M::erase_size));
    Heap<M> heap;
    const auto& r = heap.mount();
    CHECK(r.status == NvHeapStatus::empty);
    CHECK(heap.count() == 0);
    // And the heap is usable from there.
    CHECK(put<M>(heap, 7, 100, 0x11));
    CHECK(holds<M>(heap, 7, 100, 0x11));
}

TEST_CASE_TEMPLATE("a floor inside the map home REFUSES the mount", M, AvrFlash,
                   G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    REQUIRE(put<M>(heap, 1, 64, 0x22));

    // The image grew: its read-only data now reaches above the map home.
    const uint32_t erases = M::total_erases();
    const uint32_t cells = M::cells_programmed;
    zones_pages<M>(4, static_cast<uint16_t>(M::page_count / 2 - 4),
                   static_cast<uint16_t>(M::page_count - 1),
                   static_cast<uint16_t>(M::page_count));
    Heap<M> grown;
    const auto& r = grown.mount();
    CHECK(r.status == NvHeapStatus::bad_geometry);
    CHECK(!grown.mounted());
    CHECK(!grown.find(1).has_value());
    CHECK(!grown.alloc(2, 64).has_value());
    CHECK(!grown.rewrite(1).has_value());
    // A refused mount serves nothing and WRITES NOTHING.
    CHECK(M::total_erases() == erases);
    CHECK(M::cells_programmed == cells);
}

TEST_CASE_TEMPLATE("zones the wrong way up are refused too", M, AvrFlash,
                   G0Flash) {
    fresh_media<M>();
    M::set_zone(0, 4u * M::erase_size, 8u * M::erase_size);   // floor > ceiling
    Heap<M> heap;
    CHECK(heap.mount().status == NvHeapStatus::bad_geometry);
}

// ---------------------------------------------------------------------------
//  the round trip
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("a block is written, sealed, found and read back", M,
                   AvrFlash, G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);

    fill(work, 40, 0x5A);
    std::optional<typename Heap<M>::Writer> w = heap.alloc(11, 40);
    REQUIRE(w.has_value());
    CHECK(w->capacity() == M::erase_size);
    CHECK(w->record_id() == 11);
    // Not published until the seal: nothing to find yet.
    CHECK(!heap.find(11).has_value());
    CHECK(w->append(std::span<const uint8_t>(work, 40)));
    CHECK(w->written() == 40);
    CHECK(w->seal());
    CHECK(w->sealed());

    CHECK(heap.count() == 1);
    CHECK(heap.sequence() == 1);
    const std::optional<NvBlock<M>> b = heap.find(11);
    REQUIRE(b.has_value());
    CHECK(b->length == 40);
    CHECK(b->address % M::erase_size == 0);
    CHECK(b->read(0, std::span<uint8_t>(back, 40)));
    CHECK(same(work, back, 40));
    // A read that leaves the block is refused, not truncated.
    CHECK(!b->read(1, std::span<uint8_t>(back, 40)));
    CHECK(b->read(38, std::span<uint8_t>(back, 2)));
    CHECK(M::double_programs == 0);
}

TEST_CASE_TEMPLATE("a multi-page payload with a partial-cell tail survives a "
                   "remount", M, AvrFlash, G0Flash) {
    fresh_media<M>();
    const uint32_t len = M::erase_size + 5;   // two pages, ragged tail
    {
        Heap<M> heap;
        REQUIRE(heap.mount().status == NvHeapStatus::empty);
        REQUIRE(put<M>(heap, 3, len, 0x81));
        CHECK(heap.entry(0).size_pages == 2);
        CHECK(holds<M>(heap, 3, len, 0x81));
    }
    Heap<M> again;
    const auto& r = again.mount();
    CHECK(r.status == NvHeapStatus::ok);
    CHECK(r.survivors == 1);
    CHECK(r.lost == 0);
    CHECK(r.survivor_ids[0] == 3);
    CHECK(r.build_id == 0xA1B2C3D4u);
    CHECK(holds<M>(again, 3, len, 0x81));
    CHECK(M::double_programs == 0);
}

TEST_CASE_TEMPLATE("appends of any chunking build the same block", M, AvrFlash,
                   G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    const uint32_t len = 301;
    fill(work, len, 0x44);
    std::optional<typename Heap<M>::Writer> w = heap.alloc(5, len);
    REQUIRE(w.has_value());
    uint32_t at = 0;
    uint32_t chunk = 1;
    while (at < len) {
        const uint32_t n = (at + chunk > len) ? len - at : chunk;
        REQUIRE(w->append(std::span<const uint8_t>(work + at, n)));
        at += n;
        chunk = chunk * 2 + 1;
    }
    REQUIRE(w->seal());
    CHECK(holds<M>(heap, 5, len, 0x44));
}

TEST_CASE_TEMPLATE("a payload longer than the reservation is refused, and the "
                   "handle stays dead", M, AvrFlash, G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    std::optional<typename Heap<M>::Writer> w = heap.alloc(9, 10);
    REQUIRE(w.has_value());
    fill(work, M::erase_size + 1, 0x10);
    CHECK(!w->append(std::span<const uint8_t>(work, M::erase_size + 1)));
    CHECK(w->failed());
    CHECK(!w->seal());
    CHECK(heap.count() == 0);
}

TEST_CASE_TEMPLATE("an abandoned handle leaves the heap untouched and its "
                   "pages free", M, AvrFlash, G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    uint16_t page = 0;
    {
        std::optional<typename Heap<M>::Writer> w = heap.alloc(4, 20);
        REQUIRE(w.has_value());
        page = static_cast<uint16_t>(w->address() / M::erase_size);
        CHECK(w->append(std::span<const uint8_t>(work, 4)));
    }   // destroyed without a seal
    CHECK(heap.count() == 0);
    CHECK(heap.sequence() == 0);
    std::optional<typename Heap<M>::Writer> again = heap.alloc(4, 20);
    REQUIRE(again.has_value());
    CHECK(again->address() / M::erase_size == page);   // the slot came back
}

// ---------------------------------------------------------------------------
//  supersede and rewrite
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("a same-id alloc supersedes only at the seal", M, AvrFlash,
                   G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    REQUIRE(put<M>(heap, 2, 64, 0x01));
    const uint32_t old_addr = heap.find(2)->address;

    fill(work, 90, 0x02);
    std::optional<typename Heap<M>::Writer> w = heap.alloc(2, 90);
    REQUIRE(w.has_value());
    CHECK(w->address() != old_addr);        // the old copy is an obstacle
    CHECK(heap.find(2)->address == old_addr);
    CHECK(holds<M>(heap, 2, 64, 0x01));     // still the old one
    REQUIRE(w->append(std::span<const uint8_t>(work, 90)));
    CHECK(holds<M>(heap, 2, 64, 0x01));     // STILL the old one
    REQUIRE(w->seal());

    CHECK(heap.count() == 1);               // one id, one block
    CHECK(heap.find(2)->address != old_addr);
    CHECK(holds<M>(heap, 2, 90, 0x02));
    Heap<M> again;
    CHECK(again.mount().survivors == 1);
    CHECK(holds<M>(again, 2, 90, 0x02));
}

TEST_CASE_TEMPLATE("a power loss between alloc and seal leaves the old block "
                   "current", M, AvrFlash, G0Flash) {
    fresh_media<M>();
    {
        Heap<M> heap;
        REQUIRE(heap.mount().status == NvHeapStatus::empty);
        REQUIRE(put<M>(heap, 2, 64, 0x01));
        fill(work, 64, 0x02);
        std::optional<typename Heap<M>::Writer> w = heap.alloc(2, 64);
        REQUIRE(w.has_value());
        REQUIRE(w->append(std::span<const uint8_t>(work, 64)));
        M::cut_after(0);                    // the lights go out before the seal
        CHECK(!w->seal());
    }
    M::power_on();
    Heap<M> again;
    const auto& r = again.mount();
    CHECK(r.status == NvHeapStatus::ok);
    CHECK(r.survivors == 1);
    CHECK(r.lost == 0);
    CHECK(holds<M>(again, 2, 64, 0x01));
}

TEST_CASE_TEMPLATE("rewrite keeps the address and updates the contents", M,
                   AvrFlash, G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    REQUIRE(put<M>(heap, 6, 200, 0x30));
    const uint32_t addr = heap.find(6)->address;

    fill(work, 120, 0x40);
    std::optional<typename Heap<M>::Writer> w = heap.rewrite(6);
    REQUIRE(w.has_value());
    CHECK(w->address() == addr);
    CHECK(w->capacity() == M::erase_size);
    REQUIRE(w->append(std::span<const uint8_t>(work, 120)));
    REQUIRE(w->seal());
    CHECK(heap.count() == 1);
    CHECK(heap.find(6)->address == addr);
    CHECK(holds<M>(heap, 6, 120, 0x40));

    Heap<M> again;
    CHECK(again.mount().survivors == 1);
    CHECK(again.find(6)->address == addr);
    CHECK(holds<M>(again, 6, 120, 0x40));
    CHECK(!heap.rewrite(77).has_value());   // no such block
    CHECK(M::double_programs == 0);
}

TEST_CASE_TEMPLATE("a crash mid-rewrite is REPORTED as a loss, not hidden", M,
                   AvrFlash, G0Flash) {
    fresh_media<M>();
    {
        Heap<M> heap;
        REQUIRE(heap.mount().status == NvHeapStatus::empty);
        REQUIRE(put<M>(heap, 6, 200, 0x30));
        REQUIRE(put<M>(heap, 7, 50, 0x60));
        fill(work, 120, 0x40);
        std::optional<typename Heap<M>::Writer> w = heap.rewrite(6);
        REQUIRE(w.has_value());
        REQUIRE(w->append(std::span<const uint8_t>(work, 60)));
        M::cut_after(0);
        CHECK(!w->append(std::span<const uint8_t>(work + 60, 60)));
    }
    M::power_on();
    Heap<M> again;
    const auto& r = again.mount();
    CHECK(r.status == NvHeapStatus::ok);
    CHECK(r.lost == 1);
    CHECK(r.lost_ids[0] == 6);
    CHECK(r.survivors == 1);
    CHECK(r.survivor_ids[0] == 7);
    CHECK(!again.find(6).has_value());
    CHECK(holds<M>(again, 7, 50, 0x60));
    // The lost entry is gone from the NEXT version, and its pages are free.
    REQUIRE(put<M>(again, 8, 50, 0x70));
    Heap<M> third;
    const auto& r3 = third.mount();
    CHECK(r3.lost == 0);
    CHECK(r3.survivors == 2);
}

// ---------------------------------------------------------------------------
//  the power-cut sweep
// ---------------------------------------------------------------------------

/// The whole point of the map pair: cut the power at EVERY program unit
/// of a mutation and the heap comes back as either entirely the old
/// thing or entirely the new one - never half of either.
TEST_CASE_TEMPLATE("a supersede is atomic at every single write", M, AvrFlash,
                   G0Flash) {
    // How many program units the mutation takes, measured once.
    uint32_t units = 0;
    {
        fresh_media<M>();
        Heap<M> heap;
        REQUIRE(heap.mount().status == NvHeapStatus::empty);
        REQUIRE(put<M>(heap, 1, 70, 0xA0));
        const uint32_t before = M::cells_programmed;
        REQUIRE(put<M>(heap, 1, 130, 0xB0));
        units = M::cells_programmed - before;
    }
    REQUIRE(units > 4);

    for (uint32_t cut = 0; cut <= units; ++cut) {
        fresh_media<M>();
        {
            Heap<M> heap;
            REQUIRE(heap.mount().status == NvHeapStatus::empty);
            REQUIRE(put<M>(heap, 1, 70, 0xA0));
            M::cut_after(cut);
            fill(work, 130, 0xB0);
            std::optional<typename Heap<M>::Writer> w = heap.alloc(1, 130);
            if (w) {
                if (w->append(std::span<const uint8_t>(work, 130))) {
                    (void)w->seal();
                }
            }
        }
        M::power_on();
        Heap<M> again;
        const auto& r = again.mount();
        INFO("cut after ", cut, " of ", units, " program units");
        REQUIRE(r.status == NvHeapStatus::ok);
        REQUIRE(r.lost == 0);
        REQUIRE(r.survivors == 1);
        const bool old_one = holds<M>(again, 1, 70, 0xA0);
        const bool new_one = holds<M>(again, 1, 130, 0xB0);
        CHECK((old_one != new_one));        // one of the two, never both
        CHECK((old_one || new_one));        // and never neither
        CHECK(M::double_programs == 0);
    }
}

TEST_CASE_TEMPLATE("a new block never damages the blocks already there", M,
                   AvrFlash, G0Flash) {
    uint32_t units = 0;
    {
        fresh_media<M>();
        Heap<M> heap;
        REQUIRE(heap.mount().status == NvHeapStatus::empty);
        REQUIRE(put<M>(heap, 1, 70, 0xA0));
        const uint32_t before = M::cells_programmed;
        REQUIRE(put<M>(heap, 2, 90, 0xC0));
        units = M::cells_programmed - before;
    }

    for (uint32_t cut = 0; cut <= units; ++cut) {
        fresh_media<M>();
        {
            Heap<M> heap;
            REQUIRE(heap.mount().status == NvHeapStatus::empty);
            REQUIRE(put<M>(heap, 1, 70, 0xA0));
            M::cut_after(cut);
            (void)put<M>(heap, 2, 90, 0xC0);
        }
        M::power_on();
        Heap<M> again;
        const auto& r = again.mount();
        INFO("cut after ", cut, " of ", units, " program units");
        REQUIRE(r.lost == 0);
        CHECK(holds<M>(again, 1, 70, 0xA0));      // untouched, always
        const bool has2 = again.find(2).has_value();
        CHECK((!has2 || holds<M>(again, 2, 90, 0xC0)));
        CHECK(M::double_programs == 0);
    }
}

// ---------------------------------------------------------------------------
//  a reflash over a live block
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("a reflash that clobbers one block loses exactly that one, "
                   "and its gap comes back", M, AvrFlash, G0Flash) {
    fresh_media<M>();
    // One zone, exactly four pages: two two-page blocks fill it, so the
    // gap left by the loss is the ONLY place a new block can go.
    zones_pages<M>(8, 12, 12, 12);
    uint32_t clobbered = 0;
    {
        Heap<M> heap;
        REQUIRE(heap.mount().status == NvHeapStatus::empty);
        REQUIRE(put<M>(heap, 1, 2u * M::erase_size, 0x11));    // pages 10, 11
        REQUIRE(put<M>(heap, 2, 2u * M::erase_size, 0x22));    // pages 8, 9
        CHECK(heap.free_pages(0) == 0);
        clobbered = heap.find(1)->address;
    }
    // The new image is written over the pages block 1 was living in -
    // avrdude -D leaves everything else alone.
    fill(work, 2u * M::erase_size, 0xEE);
    M::reflash(clobbered, std::span<const uint8_t>(work, 2u * M::erase_size));

    Heap<M> heap;
    const auto& r = heap.mount();
    CHECK(r.status == NvHeapStatus::ok);
    CHECK(r.lost == 1);
    CHECK(r.lost_ids[0] == 1);
    CHECK(r.survivors == 1);
    CHECK(r.survivor_ids[0] == 2);
    CHECK(holds<M>(heap, 2, 2u * M::erase_size, 0x22));
    CHECK(heap.free_pages(0) == 2);

    // The freed pages are usable again, and they are exactly the ones
    // the lost block held.
    REQUIRE(put<M>(heap, 3, 2u * M::erase_size, 0x33));
    CHECK(heap.find(3)->address == clobbered);
    CHECK(holds<M>(heap, 3, 2u * M::erase_size, 0x33));
    CHECK(holds<M>(heap, 2, 2u * M::erase_size, 0x22));
}

TEST_CASE_TEMPLATE("a chip erase leaves an empty, mountable heap", M, AvrFlash,
                   G0Flash) {
    fresh_media<M>();
    {
        Heap<M> heap;
        REQUIRE(heap.mount().status == NvHeapStatus::empty);
        REQUIRE(put<M>(heap, 1, 100, 0x11));
    }
    M::chip_erase();
    Heap<M> heap;
    const auto& r = heap.mount();
    CHECK(r.status == NvHeapStatus::empty);
    CHECK(r.survivors == 0);
    CHECK(r.lost == 0);
    CHECK(!heap.find(1).has_value());
    CHECK(put<M>(heap, 1, 100, 0x11));
}

// ---------------------------------------------------------------------------
//  the map rotation
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("map versions rotate: seq climbs, wear is shared, the "
                   "oldest page is the one recycled", M, AvrFlash, G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    const uint32_t a = map_page_index<M>(0);
    const uint32_t b = map_page_index<M>(1);

    uint8_t last_page = 0xFF;
    for (uint32_t i = 1; i <= 6; ++i) {
        REQUIRE(put<M>(heap, 1, 50, static_cast<uint8_t>(i)));
        CHECK(heap.sequence() == i);
        CHECK(heap.map_page() != last_page);   // never the current one
        last_page = heap.map_page();
    }
    CHECK(M::erases_of_page(a) == 3);
    CHECK(M::erases_of_page(b) == 3);

    // And the newest version is the one a fresh mount picks up.
    Heap<M> again;
    CHECK(again.mount().seq == 6);
    CHECK(holds<M>(again, 1, 50, 6));
}

// ---------------------------------------------------------------------------
//  placement
// ---------------------------------------------------------------------------

TEST_CASE("blocks land top-down in the zone with the most clearance") {
    using M = AvrFlash;
    fresh_media<M>();
    // Middle zone: pages 10..14 (4 pages). Tail zone: pages 20..28 (8).
    zones_pages<M>(10, 14, 20, 28);
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);

    // Two pages fit in both; the tail leaves the block 6 pages above its
    // floor against the middle zone's 2, so the tail wins - and inside it
    // the block goes as high as it can.
    REQUIRE(put<M>(heap, 1, 2u * M::erase_size, 0x11));
    CHECK(heap.find(1)->address / M::erase_size == 26);

    // Next one goes right under it, still in the tail.
    REQUIRE(put<M>(heap, 2, 2u * M::erase_size, 0x22));
    CHECK(heap.find(2)->address / M::erase_size == 24);

    // Now a four-page block: the tail's remaining gap is 20..24, whose
    // clearance is 0; the middle zone's is 10..14, also 0 - the first
    // zone wins the tie by being first.
    REQUIRE(put<M>(heap, 3, 4u * M::erase_size, 0x33));
    CHECK(heap.find(3)->address / M::erase_size == 10);
    CHECK(heap.free_pages(0) == 0);
    CHECK(heap.free_pages(1) == 4);
}

TEST_CASE("the fallback zone takes what the best zone cannot hold") {
    using M = AvrFlash;
    fresh_media<M>();
    zones_pages<M>(10, 20, 40, 44);     // middle 10 pages, tail 4
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);

    // A six-page block does not fit the tail at all: the middle zone
    // takes it, top-down. That is the fallback.
    REQUIRE(put<M>(heap, 1, 6u * M::erase_size, 0x11));
    CHECK(heap.find(1)->address / M::erase_size == 14);
    // Two pages: the middle zone's gap leaves 2 pages of clearance and so
    // does the tail - the first zone wins the tie.
    REQUIRE(put<M>(heap, 2, 2u * M::erase_size, 0x22));
    CHECK(heap.find(2)->address / M::erase_size == 12);
    // Now the middle zone's only gap sits ON its floor (clearance 0)
    // while the tail still offers 2: the tail takes this one.
    REQUIRE(put<M>(heap, 3, 2u * M::erase_size, 0x33));
    CHECK(heap.find(3)->address / M::erase_size == 42);
    // Both remaining gaps are on their floors: the tie goes to the first
    // zone again, and that closes it.
    REQUIRE(put<M>(heap, 4, 2u * M::erase_size, 0x44));
    CHECK(heap.find(4)->address / M::erase_size == 10);
    CHECK(heap.free_pages(0) == 0);
    CHECK(heap.free_pages(1) == 2);
}

TEST_CASE("a gap between two live blocks is reused") {
    using M = AvrFlash;
    fresh_media<M>();
    zones_pages<M>(10, 16, 16, 16);     // one usable zone, six pages
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    REQUIRE(put<M>(heap, 1, 2u * M::erase_size, 0x11));   // pages 14, 15
    REQUIRE(put<M>(heap, 2, 2u * M::erase_size, 0x22));   // pages 12, 13
    REQUIRE(put<M>(heap, 3, 2u * M::erase_size, 0x33));   // pages 10, 11
    CHECK(heap.free_pages(0) == 0);

    // Lose the middle one to a reflash, then watch the hole get filled.
    const uint32_t hole = heap.find(2)->address;
    fill(work, 2u * M::erase_size, 0xEE);
    M::reflash(hole, std::span<const uint8_t>(work, 2u * M::erase_size));
    Heap<M> again;
    CHECK(again.mount().lost == 1);
    CHECK(again.free_pages(0) == 2);
    REQUIRE(put<M>(again, 4, 2u * M::erase_size, 0x44));
    CHECK(again.find(4)->address == hole);
    CHECK(holds<M>(again, 1, 2u * M::erase_size, 0x11));
    CHECK(holds<M>(again, 3, 2u * M::erase_size, 0x33));
}

// ---------------------------------------------------------------------------
//  refusals
// ---------------------------------------------------------------------------

TEST_CASE("the block table is a declared limit, and it is enforced") {
    using M = AvrFlash;
    using Small = NvHeap<M, 2, 2>;
    fresh_media<M>();
    Small heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);

    fill(work, 30, 0x01);
    std::optional<Small::Writer> a = heap.alloc(1, 30);
    REQUIRE(a.has_value());
    REQUIRE(a->append(std::span<const uint8_t>(work, 30)));
    REQUIRE(a->seal());
    std::optional<Small::Writer> b = heap.alloc(2, 30);
    REQUIRE(b.has_value());
    // One live, one unsealed: the table is full.
    CHECK(!heap.alloc(3, 30).has_value());
    REQUIRE(b->append(std::span<const uint8_t>(work, 30)));
    REQUIRE(b->seal());
    CHECK(heap.count() == 2);
    CHECK(!heap.alloc(3, 30).has_value());
    // Superseding an id that is already there needs no new slot.
    CHECK(heap.alloc(1, 30).has_value());
}

TEST_CASE("what does not fit is refused, and so is a block of nothing") {
    using M = AvrFlash;
    fresh_media<M>();
    zones_pages<M>(10, 14, 14, 14);     // four pages, full stop
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    CHECK(!heap.alloc(1, 0).has_value());                     // no such block
    CHECK(!heap.alloc(1, 5u * M::erase_size).has_value());    // bigger than the zone
    REQUIRE(put<M>(heap, 1, 3u * M::erase_size, 0x11));
    CHECK(!heap.alloc(2, 2u * M::erase_size).has_value());    // no gap left
    CHECK(put<M>(heap, 2, 1u * M::erase_size, 0x22));         // the last page
    CHECK(!heap.alloc(3, 1).has_value());                     // nothing left at all
    CHECK(heap.free_pages(0) == 0);
}

TEST_CASE("an unmounted heap refuses everything") {
    using M = AvrFlash;
    fresh_media<M>();
    Heap<M> heap;
    CHECK(!heap.mounted());
    CHECK(!heap.alloc(1, 10).has_value());
    CHECK(!heap.rewrite(1).has_value());
    CHECK(!heap.find(1).has_value());
    CHECK(M::cells_programmed == 0);
    CHECK(M::total_erases() == 0);
}

// ---------------------------------------------------------------------------
//  the ECC discipline, once for the whole suite
// ---------------------------------------------------------------------------

TEST_CASE_TEMPLATE("no program unit is ever written twice between erases", M,
                   AvrFlash, G0Flash) {
    fresh_media<M>();
    Heap<M> heap;
    REQUIRE(heap.mount().status == NvHeapStatus::empty);
    for (uint16_t i = 0; i < 5; ++i) {
        REQUIRE(put<M>(heap, i, 300, static_cast<uint8_t>(i * 16)));
    }
    for (uint16_t i = 0; i < 5; ++i) {
        std::optional<typename Heap<M>::Writer> w = heap.rewrite(i);
        REQUIRE(w.has_value());
        fill(work, 200, static_cast<uint8_t>(i + 1));
        REQUIRE(w->append(std::span<const uint8_t>(work, 200)));
        REQUIRE(w->seal());
    }
    for (uint16_t i = 0; i < 5; ++i) {
        CHECK(holds<M>(heap, i, 200, static_cast<uint8_t>(i + 1)));
    }
    CHECK(M::double_programs == 0);
    CHECK(M::misaligned == 0);
}
