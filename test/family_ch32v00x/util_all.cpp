// The whole of kernel/ and util/ over this target's platform, in one
// translation unit: every header included, every service instantiated
// with a fake for the target half it needs. Two things are proven at
// once - that the composition holds on a platform with sixteen
// registers and atomic_width 4, and that WCH's gcc 15.2 accepts every
// C++23 construct these headers are written with (the other three
// targets compile on gcc 16.2; the freestanding libstdc++ pieces brio
// leans on - variant, optional, span, concepts, bit, expected - are
// what this file exercises through the services).
//
// Instantiation only: no main(), nothing linked, RAM is not a concern
// (a fake flash of the journal's geometry would not fit 8 KB and does
// not have to).
#include <stdint.h>
#include <string.h>

#include <array>
#include <optional>
#include <span>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"

#include "kernel/active_object.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
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
#include "util/rgb_lamp.hpp"
#include "util/ring.hpp"
#include "util/serial_port.hpp"
#include "util/spi_bus.hpp"
#include "util/stream.hpp"
#include "util/testbench.hpp"
#include "util/timestamp.hpp"
#include "util/trace.hpp"
#include "util/wire.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;
using Serial = Uart<1, P>;

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
    static constexpr uint32_t erase_size = 1024;    // this family's sector
    static constexpr uint32_t write_cell = 2;
    static constexpr uint32_t flash_end = 0x0000F800; // 62 K
    static constexpr uint8_t zone_count = 1;
    static constexpr std::array<FlashZone, 1> zones() {
        return {FlashZone{flash_end, 0x0000C000}};
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
                      SleepVote, LineReceived, NvDone> {
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
            [](NvDone d) { seen += d.status; return handled(); });
    }
};

using Sampler = AnalogSampler<FakeAdc, P, Subscribers<Listener>, PinIn<2>{}, Internal::vref>;
using Relay = BlockRelay<P, Subscribers<Listener>, FakeSource>;
using Power = PowerManager<P, FakeSite, PowerConfig{}, Listener>;
using Lines = SerialPort<Serial, P, Listener, 48>;

static_assert(ActiveObject<Sampler>);
static_assert(ActiveObject<Relay>);
static_assert(ActiveObject<Power>);
static_assert(ActiveObject<Lines>);
static_assert(ActiveObject<Writer>);

// Borrowers before lenders: the kernel checks the loan direction.
using System = Kernel<P, Listener, Sampler, Relay, Power, Lines, Writer>;

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

    (void)System::step();
    System::idle_if_empty();
}

[[noreturn]] void die() {
    panic<P>(PanicCode::queue_overflow, 1);
}
