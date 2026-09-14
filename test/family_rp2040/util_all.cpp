// The whole of kernel/ and util/ over this platform: every framework
// header instantiated once against Rp2040Platform, the proof that the
// kernel and the services compile unchanged on this family - the same
// TU the ch32v00x fixture keeps for WCH's compiler, here for the sixth
// project.
#include "rp2040/platform.hpp"

#include "kernel/active_object.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/panic.hpp"
#include "kernel/post.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/analog.hpp"
#include "util/block_stream.hpp"
#include "util/bus_master.hpp"
#include "util/clock.hpp"
#include "util/crc.hpp"
#include "util/i2c_bus.hpp"
#include "util/input_scanner.hpp"
#include "util/meter_sampler.hpp"
#include "util/nv_heap.hpp"
#include "util/nv_journal.hpp"
#include "util/nv_record.hpp"
#include "util/persistent_panic.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/pwm_channel.hpp"
#include "util/rgb_lamp.hpp"
#include "util/ring.hpp"
#include "util/serial_port.hpp"
#include "util/spi_bus.hpp"
#include "util/stream.hpp"
#include "util/testbench.hpp"
#include "util/timestamp.hpp"
#include "util/trace.hpp"
#include "util/wire.hpp"

#include "gfx/draw.hpp"

using namespace brio;
using P = Rp2040Platform<>;

struct Ping {};
struct Echo : Fsm<Echo, Ping> {
    static inline EventQueue<Event, 4, P> queue;
    static inline TimeEvent<P, Echo, Ping> tick{Ping{}};
    static void init() { start(&idle); }
    static Status idle(const Event& e) {
        return match(e, [](Entry) { tick.arm_every(ticks_from_ms<P>(10)); return handled(); },
                     [](Ping) { return handled(); }, [](auto) { return unhandled(); });
    }
};

using Loop = Tenuto<P, Echo>;
using Log = Ring<uint8_t, 64, P>;
using Latch = MeterLatch<uint16_t, P, 0>;

void util_all_verbs() {
    Loop::init_all();
    (void)Loop::step();
    post<Echo>(Ping{});
    Log log;
    (void)log.push(1);
    (void)log.pop();
    Latch::store(1);
    (void)Latch::take();
    (void)crc16(nullptr, 0);
    (void)ticks_from_ms<P>(5);
    TimeEvents<P>::process();
    (void)TimeEvents<P>::next_deadline();
}

// gfx/ is target-independent, so what this adds is the COMPILER's
// verdict: every C++23 construct the surfaces and the primitives are
// written with, accepted here too.
namespace {
using GfxPanel = Framebuffer<Mono, 128, 64>;
std::array<uint8_t, GfxPanel::bytes> gfx_bits;
GfxPanel gfx_panel{gfx_bits};
static_assert(Surface<GfxPanel> && ReadableSurface<GfxPanel>);
static_assert(Surface<Viewport<GfxPanel>> && !ReadableSurface<Viewport<GfxPanel>>);
} // namespace

void gfx_verbs() {
    clear(gfx_panel, 0);
    rect(gfx_panel, 0, 0, 128, 64, 1);
    line(gfx_panel, -20000, -20000, 20000, 20000, 1);
    Viewport<GfxPanel> window(gfx_panel, 8, 8, 48, 24);
    fill_rect(window, -5, -5, 100, 100, 1);
    const std::array<GfxPanel::Color, 4> run{1, 0, 1, 0};
    window.write_run(46, 3, run);
}
