// Family smoke TU for samc/postmortem.hpp: the MTB post-mortem store,
// its two entry paths and the boot side must COMPILE on the E, G and J
// 18A headers (tools/check_samc.sh sweeps all three).
//
// Nothing here is package-dependent - the MTB is core-private and the
// record is SRAM - so what this fixture pins is the SHAPE: the geometry
// rules that are static_asserts (their negatives live in neg/), the
// record layout, and that the reporter and the fault body compose with
// what kernel/panic.hpp and samc/reset.hpp already provide.

#include <stdint.h>
#include <span>

#include "samc/platform_sam.hpp"
#include "samc/postmortem.hpp"

using namespace brio;

using Trace = MtbPostMortem<256, 16>;

static_assert(Trace::buffer_bytes == 256);
static_assert(Trace::kept == 16);
static_assert(Mtb::packets_for(Trace::buffer_bytes) == 32,
              "the rolling buffer must hold at least what is kept");

// The record is header + packets with no padding: 4 + 2 + 1 + 1, then
// eight bytes per packet.
static_assert(sizeof(Trace::Record) == 8 + 16 * sizeof(MtbPacket));
static_assert(sizeof(MtbPacket) == 8);

// The smallest store the rules allow, and the largest keep a buffer can
// serve - both legal, and both instantiated here so the asserts run.
using Smallest = MtbPostMortem<16, 1>;
using Exact = MtbPostMortem<128, 16>;
static_assert(Smallest::kept == 1);
static_assert(Exact::kept == Mtb::packets_for(128));

/// The panic path, with the reset chained the way an app spells it.
using Reporter = TracingReporter<Trace>;
/// And the same capture in front of a reporter that does not reset -
/// the composition test an app makes when it wants the trace without
/// ending the program.
using QuietReporter = TracingReporter<Trace, trace_from_panic, HaltReporter>;

void verbs() {
    (void)Trace::arm();
    (void)Trace::buffer();
    (void)Trace::capture(trace_from_fault);
    (void)Trace::pending();
    (void)Trace::packets();
    (void)Trace::raw().count;
    (void)Trace::checksum(Trace::raw());

    if (const auto t = Trace::take()) {
        (void)t->source;
        for (const MtbPacket& p : t->packets) {
            (void)p.destination();
        }
    }
    Trace::clear();
    Trace::disarm();

    QuietReporter::report(PanicCode::assert_failed, 0);
}

[[noreturn]] void fault_body() {
    hard_fault_trace_reset<SamPlatform, Trace>(0x11);
}

[[noreturn]] void panic_path() {
    panic<SamPlatform, Reporter>(PanicCode::assert_failed, 0x22);
}
