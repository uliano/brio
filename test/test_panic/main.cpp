// Host tests for kernel/panic.hpp: breadcrumb write ordering, the
// debugger hook, and the boot-side fetch-and-clear.
// Run with: pio test -e native
//
// panic() never returns; on the host we escape it with a reporter that
// throws (the native build keeps exceptions on). Everything panic() does
// BEFORE the reporter - breadcrumb, break_here - is therefore observable.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>

#include "host/platform_host.hpp"
#include "kernel/panic.hpp"

namespace {

using brio::HostPlatform;
using brio::PanicCode;

struct Escape {};

struct ThrowingReporter {
    static inline PanicCode seen_code = PanicCode::none;
    static inline uint8_t seen_context = 0;
    static inline bool record_was_written = false;
    static inline bool break_came_first = false;

    static void report(PanicCode code, uint8_t context) {
        seen_code = code;
        seen_context = context;
        // observable ordering: by the time the reporter runs, the
        // breadcrumb is already in storage and break_here() has fired
        record_was_written =
            HostPlatform::panic_record().magic == brio::panic_magic;
        break_came_first = HostPlatform::break_calls == 1;
        throw Escape{};
    }
};

} // namespace

TEST_CASE("panic writes the breadcrumb and hits the debugger hook before reporting") {
    HostPlatform::reset();

    bool escaped = false;
    try {
        brio::panic<HostPlatform, ThrowingReporter>(PanicCode::queue_overflow, 7);
    } catch (Escape&) {
        escaped = true;
    }

    CHECK(escaped);
    CHECK(ThrowingReporter::seen_code == PanicCode::queue_overflow);
    CHECK(ThrowingReporter::seen_context == 7);
    CHECK(ThrowingReporter::record_was_written);
    CHECK(ThrowingReporter::break_came_first);
}

TEST_CASE("take_panic_record returns a valid record exactly once") {
    HostPlatform::reset();

    CHECK_FALSE(brio::take_panic_record<HostPlatform>().has_value());  // cold boot

    try {
        brio::panic<HostPlatform, ThrowingReporter>(PanicCode::assert_failed, 42);
    } catch (Escape&) {
    }

    auto rec = brio::take_panic_record<HostPlatform>();
    CHECK(rec.has_value());
    if (rec.has_value()) {
        CHECK(rec->code == static_cast<uint8_t>(PanicCode::assert_failed));
        CHECK(rec->context == 42);
    }
    CHECK_FALSE(brio::take_panic_record<HostPlatform>().has_value());  // cleared
}

TEST_CASE("garbage in the record does not masquerade as a panic") {
    HostPlatform::reset();
    HostPlatform::panic_record() = brio::PanicRecord{0xDEAD, 1, 1};

    CHECK_FALSE(brio::take_panic_record<HostPlatform>().has_value());
}
