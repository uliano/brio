// mcu: ch32x035f8
// A transport whose release can refuse - a HardwareRing behind it, its
// consume() answering bool - and that does not report the ring's skip
// epoch (rx_skips()) cannot back a LineReader: a skip between two runs
// would complete the line begun with bytes from after the gap
// (util/line_reader.hpp). Refused where the drain is compiled.
#include <stdint.h>

#include <span>

#include "ch32x035/platform.hpp"
#include "kernel/fsm.hpp"
#include "util/line_reader.hpp"
#include "util/ring.hpp"

namespace {

using P = brio::Ch32x035Platform<>;

uint8_t storage[64];

struct Channel {
    static inline volatile uint32_t count = 64;
    static inline volatile uint32_t lap = 0;
    static uint32_t remaining() { return count; }
    static uint32_t laps() { return lap; }
};

using Rx = brio::HardwareRing<storage, Channel>;

struct Refusing {
    static std::span<const uint8_t> read_span() { return Rx::read_span(); }
    static bool consume(uint32_t n) { return Rx::consume(n); }
};

struct Sink : brio::Fsm<Sink, brio::LineReceived> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static void init() { start(&only); }
    static Status only(const Event&) { return handled(); }
};

} // namespace

using Reader = brio::LineReader<Refusing, P, Sink>;

void run() {
    Reader::init();
    Reader::dispatch(brio::RxActivity{});
}
