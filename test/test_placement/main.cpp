// Host tests for util/placement.hpp: what the trampoline does with the
// body it is given (the result forwarded, the side effects through the
// captures, one out-of-line instance a lambda in SRAM's specialization),
// whichever placement it is asked for,
// and which placements the HotPlacement concept admits for which
// platform. WHERE an instance lands is the target's business - its map and
// its linker script; on the host `.fast_text.brio` is a section of the
// test binary like any other, which is what lets both specializations run
// here.
// Run with: ctest --preset host (or ctest --preset host -R test_placement)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <utility>

#include "host/platform.hpp"
#include "util/placement.hpp"

namespace {

using brio::Hot;
using brio::HotPlacement;
using brio::InFlash;
using brio::InRam;

/// A platform whose family collects `.fast_text` (the trait said true),
/// one that says false, and the host's own, which says nothing.
struct Places {
    static constexpr bool places_fast_text = true;
};
struct DoesNotPlace {
    static constexpr bool places_fast_text = false;
};
using HostPlatform = brio::HostPlatform;

/// A third tag: no specialization of Hot, no place in the concept.
struct InTightlyCoupled {};

template <class T>
concept Specialized = requires { sizeof(Hot<T>); };

static_assert(Specialized<InFlash> && Specialized<InRam> && !Specialized<InTightlyCoupled>,
              "Hot has the two specializations and no primary template");

static_assert(HotPlacement<InFlash, Places> && HotPlacement<InFlash, DoesNotPlace> &&
                  HotPlacement<InFlash, HostPlatform>,
              "InFlash is every platform's");
static_assert(HotPlacement<InRam, Places>, "InRam where the platform says it places .fast_text");
static_assert(!HotPlacement<InRam, DoesNotPlace> && !HotPlacement<InRam, HostPlatform>,
              "InRam refused where the platform says false or says nothing");
static_assert(!HotPlacement<InTightlyCoupled, Places> && !HotPlacement<InTightlyCoupled, HostPlatform>,
              "a third tag is no placement anywhere");
static_assert(brio::platform_places_fast_text<Places> && !brio::platform_places_fast_text<DoesNotPlace> &&
              !brio::platform_places_fast_text<HostPlatform>);

/// A driver's shape: a body written once, always inline, run through the
/// placement it was given - the result returned, the state touched in place.
template <class Placement>
struct Pump {
    static inline uint32_t frames = 0;
    static inline uint32_t sum = 0;
    [[gnu::always_inline]] static bool body(uint32_t in, uint32_t left) {
        sum += in;
        ++frames;
        return left == 0u;
    }
    static bool isr(uint32_t in, uint32_t left) {
        return Hot<Placement>::run([in, left] { return body(in, left); });
    }
    static void reset() {
        frames = 0;
        sum = 0;
    }
};

template <class Placement>
void runs_the_body() {
    using P = Pump<Placement>;
    P::reset();
    CHECK_FALSE(P::isr(5, 2));
    CHECK_FALSE(P::isr(7, 1));
    CHECK(P::isr(11, 0));
    CHECK(P::frames == 3u);
    CHECK(P::sum == 23u);
}

}  // namespace

TEST_CASE("Hot<InFlash> runs the body: the result forwarded, the state touched") { runs_the_body<InFlash>(); }

TEST_CASE("Hot<InRam> runs the same body the same way") { runs_the_body<InRam>(); }

TEST_CASE("a void body, its effects through a reference capture") {
    uint32_t budget = 10;
    Hot<InFlash>::run([&budget] { budget -= 3u; });
    Hot<InRam>::run([&budget] { budget -= 4u; });
    CHECK(budget == 3u);
}

TEST_CASE("the result type is the body's own") {
    const uint16_t a = Hot<InFlash>::run([] { return static_cast<uint16_t>(0xBEEFu); });
    const uint64_t b = Hot<InRam>::run([] { return uint64_t{0x1'0000'0001ULL}; });
    CHECK(a == 0xBEEFu);
    CHECK(b == 0x1'0000'0001ULL);
}

TEST_CASE("InRam: one instance a lambda - two paths are two functions, one path called twice is one") {
    // The instance's address is the function a vector would call: distinct
    // lambdas, distinct instances; the same lambda type, the same instance.
    auto first = [] { return 1; };
    auto second = [] { return 2; };
    using F1 = decltype(first);
    using F2 = decltype(second);
    auto* p1 = &Hot<InRam>::run<F1>;
    auto* p2 = &Hot<InRam>::run<F2>;
    auto* p1_again = &Hot<InRam>::run<F1>;
    CHECK(reinterpret_cast<void*>(p1) != reinterpret_cast<void*>(p2));
    CHECK(reinterpret_cast<void*>(p1) == reinterpret_cast<void*>(p1_again));
    CHECK(p1(std::move(first)) == 1);
    CHECK(p2(std::move(second)) == 2);
}
