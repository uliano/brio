// Host tests for util/analog_sampler.hpp: the input walk, attribution by
// the converter's reported input code, publication to the subscribers,
// the software pace, unknown codes. Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <vector>

#include "host/platform_host.hpp"
#include "kernel/time.hpp"
#include "util/analog_sampler.hpp"

namespace {

using brio::AnalogSample;
using brio::HostPlatform;
using brio::Sampled;

// A fake converter with a tiny input vocabulary: pin tags as types,
// internal sources as an enum - the shape of the AVR driver.
template <uint8_t n>
struct PinIn { static constexpr uint8_t code = n; };
enum class Internal : uint8_t { temp = 0x42, vdd = 0x44 };

struct FakeAdc {
    static inline uint8_t mux = 0xFF;
    static inline unsigned starts = 0;
    template <uint8_t n>
    static void select(PinIn<n>) { mux = n; }
    static void select(Internal i) { mux = static_cast<uint8_t>(i); }
    template <uint8_t n>
    static constexpr uint8_t input_code(PinIn<n>) { return n; }
    static constexpr uint8_t input_code(Internal i) { return static_cast<uint8_t>(i); }
    static void start() { ++starts; }
    static uint8_t selected() { return mux; }
};

// Two subscribers, recording what they get.
template <int tag>
struct Sub : brio::Fsm<Sub<tag>, AnalogSample> {
    using Base = brio::Fsm<Sub<tag>, AnalogSample>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline brio::EventQueue<Event, 8, HostPlatform> queue;
    static inline std::vector<AnalogSample> got;
    static void init() { Base::start(&only); }
    static void dispatch(const Event& e) { Base::dispatch(e); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](AnalogSample s) { got.push_back(s); return Base::handled(); },
            [](auto) { return Base::handled(); });
    }
};
using SubA = Sub<0>;
using SubB = Sub<1>;

using Sampler = brio::AnalogSampler<FakeAdc, HostPlatform, brio::Subscribers<SubA, SubB>,
                                    PinIn<7>{}, Internal::temp, Internal::vdd>;

// The ISR glue, as an app would write it: the converter "finishes" a
// conversion on the selected input with the given value.
void convert(uint16_t value) {
    brio::post<Sampler>(Sampled{value, FakeAdc::selected()});
}

void run_scheduler() {
    for (;;) {
        if (auto e = SubA::queue.pop()) {
            SubA::dispatch(*e);
        } else if (auto e2 = SubB::queue.pop()) {
            SubB::dispatch(*e2);
        } else if (auto s = Sampler::queue.pop()) {
            Sampler::dispatch(*s);
        } else {
            break;
        }
    }
}

void reset() {
    HostPlatform::reset();
    brio::TimeEvents<HostPlatform>::clear_all();
    FakeAdc::mux = 0xFF;
    FakeAdc::starts = 0;
    SubA::got.clear();
    SubB::got.clear();
    while (SubA::queue.pop().has_value()) {}
    while (SubB::queue.pop().has_value()) {}
    while (Sampler::queue.pop().has_value()) {}
    SubA::init();
    SubB::init();
    Sampler::init();
}

} // namespace

TEST_CASE("init selects the first input; each result walks the list") {
    reset();
    CHECK(FakeAdc::mux == 7);
    convert(100);
    run_scheduler();
    CHECK(FakeAdc::mux == 0x42);           // next input selected after the result
    convert(200);
    run_scheduler();
    CHECK(FakeAdc::mux == 0x44);
    convert(300);
    run_scheduler();
    CHECK(FakeAdc::mux == 7);              // wrapped
    REQUIRE(SubA::got.size() == 3);
    CHECK(SubA::got[0].index == 0); CHECK(SubA::got[0].value == 100);
    CHECK(SubA::got[1].index == 1); CHECK(SubA::got[1].value == 200);
    CHECK(SubA::got[2].index == 2); CHECK(SubA::got[2].value == 300);
    CHECK(SubB::got.size() == 3);          // both subscribers, by value
}

TEST_CASE("attribution follows the reported code, not the dispatch order") {
    reset();
    // Two results queued before the sampler runs (a hardware pace faster
    // than the dispatch): the second was taken on the SAME input, since
    // nobody selected the next one yet - and is labelled so.
    convert(10);
    convert(11);
    run_scheduler();
    REQUIRE(SubA::got.size() == 2);
    CHECK(SubA::got[0].index == 0);
    CHECK(SubA::got[1].index == 0);
    CHECK(FakeAdc::mux == 0x42);           // the walk resumes from the last seen
}

TEST_CASE("a code outside the list is dropped and counted") {
    reset();
    brio::post<Sampler>(Sampled{5, 0x30});
    run_scheduler();
    CHECK(SubA::got.empty());
    CHECK(Sampler::unknown_inputs() == 1);
    CHECK(FakeAdc::mux == 0x42);           // the walk still advances
}

TEST_CASE("software pace: one start per period") {
    reset();
    Sampler::start_every(brio::ticks_from_ms<HostPlatform>(10));
    CHECK(Sampler::running_every());
    for (int i = 0; i < 35; ++i) {
        ++HostPlatform::ticks;
        brio::TimeEvents<HostPlatform>::process();
        run_scheduler();
    }
    CHECK(FakeAdc::starts == 3);
    Sampler::stop();
    CHECK(!Sampler::running_every());
    for (int i = 0; i < 20; ++i) {
        ++HostPlatform::ticks;
        brio::TimeEvents<HostPlatform>::process();
        run_scheduler();
    }
    CHECK(FakeAdc::starts == 3);
}
