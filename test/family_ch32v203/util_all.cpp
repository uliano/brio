// The whole of kernel/ and util/ over this target's platform, in one
// translation unit: every header included, every service instantiated
// with a fake for the target half it needs. Two things are proven at
// once - that the composition holds on this platform, and that WCH's gcc
// 15.2 accepts every C++23 construct these headers are written with on
// the ilp32 ABI (the CH32V00x fixture proves the same headers on that
// family's ilp32e, and the other targets compile on gcc 16.2; the
// freestanding libstdc++ pieces brio leans on - variant, optional, span,
// concepts, bit, expected - are what this file exercises through the
// services).
//
// Instantiation only: no main(), nothing linked, RAM is not a concern.
// The serial port is the instance the PART offers, which on the smallest
// package is USART2 and not USART1.
#include <stdint.h>
#include <string.h>

#include <array>
#include <optional>
#include <span>

#include "ch32v203/clock.hpp"
#include "ch32v203/delay.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/ticker.hpp"
#include "ch32v203/usart.hpp"

#include "kernel/active_object.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"

#include "util/analog.hpp"
#include "util/analog_sampler.hpp"
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
#include "util/nv_writer.hpp"
#include "util/persistent_panic.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/pwm_channel.hpp"
#include "util/quadrature.hpp"
#include "util/rgb_lamp.hpp"
#include "util/ring.hpp"
#include "util/serial_port.hpp"
#include "util/spi_bus.hpp"
#include "util/stream.hpp"
#include "util/testbench.hpp"
#include "util/timestamp.hpp"
#include "util/trace.hpp"
#include "util/wire.hpp"

#include "gfx/counting.hpp"
#include "gfx/draw.hpp"
#include "gfx/font_5x7.hpp"
#include "gfx/pen.hpp"
#include "gfx/text.hpp"

using namespace brio;

using P = Ch32v203Platform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;
/// USART1 where the package bonds its pads, USART2 otherwise: the one
/// instance every part of this family offers.
using Serial = Uart<device::has_usart(1) ? 1 : 2, P>;

static_assert(Platform<P>);

// ---- storage: an NvStore with a ready interrupt, a FlashMedia ---------------
struct FakeStore {
    static constexpr uint16_t bytes = 64;
    static inline uint8_t cells[bytes];
    static constexpr uint16_t size() { return bytes; }
    static uint8_t read(uint16_t a) { return a < bytes ? cells[a] : 0xFF; }
    static bool write(uint16_t a, uint8_t v) {
        if (a >= bytes) { return false; }
        cells[a] = v;
        return true;
    }
    static bool ready() { return true; }
    static bool wait_ready() { return true; }
    static void finish() {}
    static void arm_ready_interrupt(bool) {}
    static bool ready_flag() { return true; }
    static void clear_ready_flag() {}
};
static_assert(NvStore<FakeStore>);
static_assert(NvPacedStore<FakeStore>);

struct FakeFlash {
    static constexpr uint32_t erase_size = 4096;   // this family's protection grain
    static constexpr uint32_t write_cell = device::flash_page_bytes;
    static constexpr uint32_t flash_end = device::flash_bytes;
    static constexpr uint8_t zone_count = 1;
    static constexpr std::array<FlashZone, 1> zones() {
        return {FlashZone{flash_end, flash_end - 4u * erase_size}};
    }
    static void read(uint32_t, std::span<uint8_t> dst) { memset(dst.data(), 0xFF, dst.size()); }
    static bool program(uint32_t, std::span<const uint8_t>) { return true; }
    static bool erase(uint32_t) { return true; }
    static uint32_t build_id() { return 1; }
};
static_assert(FlashMedia<FakeFlash>);

struct Settings {
    uint16_t baud_code;
    uint8_t brightness;
};

using Record = NvRecord<Settings, FakeStore>;
using Panics = PersistentPanic<FakeStore, 32>;
using Writer = NvWriter<FakeStore, P>;
using Heap = NvHeap<FakeFlash, 4, 2>;
Heap heap;
using Journal = NvJournal<FakeFlash, 4, 16, 1>;
Journal journal;
using Reporter = JournalPanic<journal, 0>;

// ---- analog: a converter with two kinds of input ----------------------------
template <uint8_t n>
struct PinIn {};
enum class Internal : uint8_t { vref = 8, vcal = 10 };

struct FakeAdc {
    static inline uint8_t mux = 0xFF;
    template <uint8_t n>
    static void select(PinIn<n>) { mux = n; }
    static void select(Internal i) { mux = static_cast<uint8_t>(i); }
    template <uint8_t n>
    static constexpr uint8_t input_code(PinIn<n>) { return n; }
    static constexpr uint8_t input_code(Internal i) { return static_cast<uint8_t>(i); }
    static void start() {}
    static uint8_t selected() { return mux; }
};
static_assert(AnalogConverter<FakeAdc>);

// ---- a block source ----------------------------------------------------------
struct FakeSource {
    using element = uint16_t;
    static inline std::array<uint16_t, 8> buf{};
    static volatile uint16_t* ready() { return buf.data(); }
    static uint16_t ready_length() { return 8; }
    static bool release() { return true; }
    static uint32_t laps() { return 0; }
    static uint32_t overruns() { return 0; }
    static bool stalled() { return false; }
};
static_assert(BlockSource<FakeSource>);

// ---- a sleep site --------------------------------------------------------------
struct FakeSite {
    static inline SleepDepth state = SleepDepth::none;
    static bool arm(SleepDepth d) { state = d; return true; }
    static void disarm() { state = SleepDepth::none; }
    static SleepDepth armed() { return state; }
};
static_assert(SleepSite<FakeSite>);

// ---- the subscribers ----------------------------------------------------------
struct Listener : Fsm<Listener, AnalogSample, BlockReady<uint16_t>, PrepareSleep, WakeReport,
                      SleepVote, LineReceived, NvDone, Turned> {
    static inline EventQueue<Event, 8, P> queue;
    static inline uint32_t seen = 0;

    static void init() { start(&only); }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](AnalogSample s) { seen += s.value; return handled(); },
            [](BlockReady<uint16_t> b) { seen += b.length; return handled(); },
            [](const PrepareSleep& p) { p.reply.send(SleepVote{true}); return handled(); },
            [](WakeReport) { return handled(); },
            [](SleepVote) { return handled(); },
            [](LineReceived l) { seen += static_cast<uint32_t>(l.line.get()[0]); return handled(); },
            [](NvDone d) { seen += d.status; return handled(); },
            [](Turned t) { seen += static_cast<uint32_t>(t.detents); return handled(); });
    }
};

using Sampler = AnalogSampler<FakeAdc, P, Subscribers<Listener>, PinIn<2>{}, Internal::vref>;
using Relay = BlockRelay<P, Subscribers<Listener>, FakeSource>;
using Power = PowerManager<P, FakeSite, PowerConfig{}, Listener>;
using Lines = SerialPort<Serial, P, Listener, 48>;

// Two contacts a knob would drive, and the decoder over them: pure util,
// so what this proves is that the compiler takes it.
struct PadA { static bool read() { return false; } };
struct PadB { static bool read() { return true; } };
using Knob = Quadrature<P, Subscribers<Listener>, PadA, PadB>;

static_assert(ActiveObject<Knob>);
static_assert(ActiveObject<Sampler>);
static_assert(ActiveObject<Relay>);
static_assert(ActiveObject<Power>);
static_assert(ActiveObject<Lines>);
static_assert(ActiveObject<Writer>);

// Borrowers before lenders: the kernel checks the loan direction.
using System = Tenuto<P, Listener, Sampler, Relay, Power, Lines, Writer>;

// ---- the pure arithmetic, at compile time ---------------------------------------
static_assert(adc_mv(2048, 4096, 3300) == 1650);
constexpr uint8_t three[3] = {0x12, 0x34, 0x56};
static_assert(load_be24(three) == 0x123456);
static_assert(load_be24_signed(three) == 0x123456);
static_assert(crc16(three, 3) != 0);

using Bench = TestBench<Serial, 8>;

void letter_a() {}

void util_verbs() {
    constexpr SysClock clock;
    constexpr Serial serial;
    (void)Serial::init(clock, 115200);
    System::init_all();

    // storage
    (void)Record::load();
    (void)Record::store(Settings{417, 128});
    (void)Record::valid();
    (void)Record::clear();
    Panics::report(PanicCode::queue_overflow, 3);
    (void)Panics::peek();
    (void)Panics::take();
    (void)heap.mount();
    (void)journal.mount();
    (void)journal.save(0, std::span<const uint8_t>{});
    (void)journal.save_reserved(1, std::span<const uint8_t>{});
    Reporter::report(PanicCode::queue_overflow, 0);
    static const uint8_t bytes[4] = {1, 2, 3, 4};
    post<Writer>(NvWrite{0, Borrowed<const uint8_t, Lease::reply>{bytes}, 4, reply_to<Listener, NvDone>()});
    post<Writer>(NvReady{});

    // analog and blocks
    Sampler::start_every(16);
    post<Sampler>(Sampled{2048, FakeAdc::selected()});
    post<Relay>(BlockDone{});
    (void)Relay::published();
    (void)Relay::laps(0);

    // power
    (void)Power::restrict(SleepDepth::light);
    (void)Power::ceiling();
    (void)Power::armed_depth();
    post<Power>(SleepRequested{SleepDepth::deep, reply_to<Listener, SleepVote>()});

    // lines, parsers, the router, the bench
    post<Lines>(RxActivity{});
    LineAssembler<32> assembler;
    (void)assembler.push('x');
    ConsoleCommandParser<4>::CommandType cmd;
    char text[] = "LED ON";
    (void)ConsoleCommandParser<4>::parse(text, cmd);
    ScpiCommandParser<4>::CommandType scpi;
    char query[] = "MEAS:VOLT? 1";
    (void)ScpiCommandParser<4>::parse(query, scpi);
    (void)command_equals(cmd.arguments[0], "ON");
    Bench bench;
    (void)bench.letter('a', "a letter", letter_a);
    bench.verdict("thing", true);
    bench.end_letter();

    // printing, time, wire, ring
    TimeStamp stamp{};
    Ticker::now(stamp);
    print(serial, "t=", stamp, " ", hex(0xBEEFu), " ", fixed(1.5f, 6, 2), " ", sci(1e-3f), " ", true, crlf);
    uint8_t wire[4];
    store_be32(wire, 0x01020304u);
    (void)load_be16(wire);
    Ring<uint16_t, 16, P> ring;
    (void)ring.push(1);
    (void)ring.pop();

    (void)delay_us(clock, 10);
    (void)System::step();
    System::idle_if_empty();
}

[[noreturn]] void die() {
    panic<P>(PanicCode::queue_overflow, 1);
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
    circle(gfx_panel, 64, 32, 30, 1);
    fill_circle(gfx_panel, 64, 32, 20, 1);
    round_rect(gfx_panel, 2, 2, 124, 60, 12, 1);
    fill_round_rect(gfx_panel, 20, 20, 40, 24, 200, 1);
    static_assert(isqrt(65535UL * 65535UL) == 65535u);
    Knob::init(1);
    (void)Knob::queue.pop();
    Knob::stop();
    (void)Knob::lost();
    static_assert(quadrature_step(0b00, 0b01).value() == 1);
    static_assert(!quadrature_step(0b00, 0b11).has_value());
    text<Font5x7>(gfx_panel, 2, 2, "brio", 1, 0);
    text_field<Font5x7>(gfx_panel, 2, 12, "3.30", 8, 1, 0);
    Pen<GfxPanel> pen(gfx_panel, 1, 0);
    pen.move_to(4, 30);
    pen.line_to(60, 44);
    pen.circle(30, 30, 6);
    pen.text<Font5x7>("V=");
    pen.text_field<Font5x7>("12.5", 6);
}
