// Host tests for the DCS link (brio/devices/dcs_link.hpp) and the
// simulated SPI host it is driven over (brio/host/sim_spi_host.hpp).
//
// TWO THINGS ARE JUDGED HERE, and the order matters. First the SIM
// itself, against the contract it claims to satisfy: the two phases in
// one select window, the D/C choreography, a null buffer, a select line
// with nothing on it, a frame of two bytes, the two completion styles -
// and the arbiter of util/bus_master.hpp running over it, which is what
// proves it is an engine and not a mock. Then the LINK over that host
// against the simulated ILI9481, with the bench's own numbers as the
// expected values: the same letters test/test_ili9481 drives through the
// framing adapter by hand, now driven through a Request by the code a
// panel driver will use.
//
// Run with: ctest --preset host (or ctest --preset host -R test_dcs_link)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include <array>
#include <span>
#include <vector>

#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "devices/ili9481.hpp"
#include "host/platform.hpp"
#include "host/sim_dcs_panel.hpp"
#include "host/sim_spi_host.hpp"
#include "kernel/tenuto.hpp"
#include "util/spi_bus.hpp"

using namespace brio;

namespace {

using Host = SimSpiHost<0>;
using Link = DcsSerialLink<Host>;

// The one claim the concept makes about this realization, made where a
// Host can be named at all.
static_assert(DcsLink<Link>);

using Panel = SimDcsPanel<Ili9481>;
using Front = SimDcsSerial<Panel>;
static_assert(SimSpiDeviceModel<Front>);

/// The select lines of this imaginary board.
constexpr SimSpiPinRef panel_cs{2};
constexpr SimSpiPinRef other_cs{4};
constexpr SimSpiPinRef dc_pin{3};
constexpr SimSpiPinRef empty_cs{5};

/// One panel for the whole suite: its frame memory is 460800 bytes and
/// has no business on a stack.
Panel& panel() {
    static Panel p{};
    return p;
}

Front& front() {
    static Front f{panel()};
    return f;
}

/// Anything that is not a panel, to prove the select line chooses.
struct CountingDevice {
    uint32_t selected = 0;
    uint32_t released = 0;
    uint32_t bytes = 0;
    uint8_t last_mosi = 0;
    bool dc_high = false;
    uint8_t answer = 0xA5;

    void select(bool low) {
        if (low) {
            ++selected;
        } else {
            ++released;
        }
    }
    void dc(bool high) { dc_high = high; }
    uint8_t byte(uint8_t mosi) {
        ++bytes;
        last_mosi = mosi;
        return answer;
    }
};

CountingDevice& other() {
    static CountingDevice d{};
    return d;
}

/// A bus with the panel on one select line and the counting device on
/// another, nothing counted and nothing remembered.
void fresh() {
    Host::release();
    Host::completion(SimSpiCompletion::immediate);
    SimSpiBench::reset();
    panel().hard_reset();
    panel().reset_counters();
    front().reset_counters();
    front().select(false);
    other() = CountingDevice{};
    REQUIRE(Host::attach(panel_cs, front()));
    REQUIRE(Host::attach(other_cs, other()));
    Host::reset_counters();
    SimSpiBench::reset();
}

/// A request of this bus, filled the way an application fills one.
Host::Request request(SimSpiPinRef cs, bool polled = true) {
    Host::Request r{};
    r.cs = cs;
    r.dc = dc_pin;
    r.clock = SimSpiClock::div4;
    r.mode = SimSpiMode::mode0;
    r.bits = SimSpiDataSize::bits8;
    r.polled = polled;
    return r;
}

/// The prototype pair a panel's application would hand the link: the
/// write tenure and the slower read one.
Link::Config link_config() {
    Host::Request w = request(panel_cs);
    w.clock = SimSpiClock::div4;
    Host::Request rd = request(panel_cs);
    rd.clock = SimSpiClock::div16;
    rd.cs_setup_us = 1;
    return Link::Config{w, rd};
}

/// A pixel's colour from its position, the pattern test_ili9481 uses:
/// every byte moves with x and y, so a stream one byte out of step
/// disagrees at once. Already masked to what eighteen bits keep.
constexpr uint32_t pattern(uint16_t x, uint16_t y) {
    uint32_t v = static_cast<uint32_t>(x) * 0x9E3779B1u;
    v ^= (static_cast<uint32_t>(y) + 1u) * 0x85EBCA77u;
    v ^= v >> 15;
    v *= 0x2C1B3C6Du;
    v ^= v >> 12;
    return v & 0x00FCFCFCu;
}

void wake(Link& link) {
    REQUIRE(link.command(Dcs::slpout, {}));
    REQUIRE(link.command(Dcs::dispon, {}));
}

void set_window(Link& link, uint16_t cs, uint16_t ce, uint16_t ps, uint16_t pe) {
    const std::array<uint8_t, 4> c = dcs_window(cs, ce);
    const std::array<uint8_t, 4> p = dcs_window(ps, pe);
    REQUIRE(link.command(Dcs::caset, c));
    REQUIRE(link.command(Dcs::paset, p));
}

}   // namespace

// =============================================================================
// the simulated host alone
// =============================================================================

TEST_CASE("a request reaches the device on the select line it names and no other") {
    fresh();
    const uint8_t cmd[1] = {0x11};
    Host::Request r = request(other_cs);
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.cmd_len = 1;
    CHECK(Host::start(r));

    CHECK(other().selected == 1);
    CHECK(other().released == 1);
    CHECK(other().bytes == 1);
    CHECK(other().last_mosi == 0x11);
    CHECK(front().transactions() == 0);
    CHECK(panel().commands_taken() == 0);
    CHECK(Host::unaddressed() == 0);
}

TEST_CASE("the select falls before the first byte and rises after the last") {
    fresh();
    const uint8_t cmd[1] = {Dcs::caset};
    const uint8_t data[4] = {0, 0, 1, 0x3F};
    Host::Request r = request(panel_cs);
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(data));
    r.len = 4;
    CHECK(Host::start(r));

    // The select is active LOW, so clear() asserts it and set() releases.
    CHECK(SimSpiBench::clears[panel_cs.pin] == 1);
    CHECK(SimSpiBench::sets[panel_cs.pin] == 1);
    CHECK(SimSpiBench::clear_at[panel_cs.pin] < Host::first_byte_at());
    CHECK(SimSpiBench::set_at[panel_cs.pin] > Host::last_byte_at());
    CHECK(SimSpiBench::level[panel_cs.pin]);   // idle high afterwards

    // D/C is LOW for exactly cmd_len frames and HIGH for len.
    REQUIRE(Host::trace_count() == 5);
    CHECK_FALSE(Host::trace(0).dc);
    CHECK(Host::trace(0).mosi == Dcs::caset);
    for (uint16_t i = 1; i < 5u; ++i) {
        CHECK(Host::trace(i).dc);
        CHECK(Host::trace(i).mosi == data[i - 1u]);
        CHECK(Host::trace(i).cs == panel_cs.pin);
    }
    // And the pin carried it: driven low before the command, high before
    // the data, one change each.
    CHECK(SimSpiBench::clears[dc_pin.pin] == 1);
    CHECK(SimSpiBench::sets[dc_pin.pin] == 1);
    CHECK(Host::bytes_clocked() == 5);
    CHECK(Host::transactions() == 1);
}

TEST_CASE("the setup time is counted and never spent, a null tx clocks 0xFF, a null rx discards") {
    fresh();
    const uint8_t cmd[1] = {Dcs::ramrd};
    uint8_t in[3] = {0, 0, 0};
    Host::Request r = request(other_cs);
    r.cs_setup_us = 7;
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.cmd_len = 1;
    r.rx = lend<Lease::reply>(in);
    r.len = 3;
    CHECK(Host::start(r));

    CHECK(Host::waited_us() == 7);   // counted; no test can measure it spent
    // tx null: the engine clocks dummies.
    CHECK(Host::trace(1).mosi == 0xFF);
    CHECK(Host::trace(3).mosi == 0xFF);
    // rx set: what the device answered.
    CHECK(in[0] == 0xA5);
    CHECK(in[2] == 0xA5);

    // The same transaction with rx null: the bytes are clocked and
    // discarded, and nothing else changes.
    Host::reset_counters();
    r.rx = {};
    r.cs_setup_us = 0;
    CHECK(Host::start(r));
    CHECK(Host::bytes_clocked() == 4);
    CHECK(Host::waited_us() == 0);
}

TEST_CASE("a select line with no device reads 0xFF and is counted, the status staying ok") {
    fresh();
    const uint8_t cmd[1] = {0x04};
    uint8_t in[4] = {0, 0, 0, 0};
    Host::Request r = request(empty_cs);
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.cmd_len = 1;
    r.rx = lend<Lease::reply>(in);
    r.len = 4;
    CHECK(Host::start(r));

    CHECK(Host::unaddressed() == 1);
    CHECK(Host::transactions() == 1);
    CHECK(Host::status() == spi_ok);   // a bus with nothing on it is not a fault
    for (uint8_t b : in) {
        CHECK(b == 0xFF);
    }
    // The select was still driven: the engine does not know what is out
    // there, and neither does a real one.
    CHECK(SimSpiBench::clears[empty_cs.pin] == 1);
    CHECK(SimSpiBench::sets[empty_cs.pin] == 1);
}

TEST_CASE("a sixteen-bit frame clocks two bytes, the high one first") {
    fresh();
    const uint8_t cmd[2] = {0x12, 0x34};    // ONE frame
    const uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};   // TWO frames
    Host::Request r = request(other_cs);
    r.bits = SimSpiDataSize::bits16;
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(data));
    r.len = 2;
    CHECK(Host::start(r));

    // cmd_len and len are FRAMES, so six bytes went out in the buffer's
    // own order - the high byte of each frame first.
    CHECK(Host::bytes_clocked() == 6);
    REQUIRE(Host::trace_count() == 6);
    CHECK(Host::trace(0).mosi == 0x12);
    CHECK(Host::trace(1).mosi == 0x34);
    CHECK_FALSE(Host::trace(1).dc);   // still the command phase
    CHECK(Host::trace(2).mosi == 0xAA);
    CHECK(Host::trace(5).mosi == 0xDD);
    CHECK(Host::trace(2).dc);
    CHECK(other().bytes == 6);
}

TEST_CASE("in deferred mode an unpolled request is held until finish, a polled one is not") {
    fresh();
    Host::completion(SimSpiCompletion::deferred);
    const uint8_t cmd[1] = {Dcs::nop};

    Host::Request async = request(other_cs, false);
    async.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    async.cmd_len = 1;
    CHECK_FALSE(Host::start(async));   // "the engine runs on its interrupt"
    CHECK(Host::pending());
    CHECK(other().bytes == 0);
    CHECK(Host::transactions() == 0);

    CHECK(Host::finish() == spi_ok);
    CHECK_FALSE(Host::pending());
    CHECK(other().bytes == 1);
    CHECK(Host::transactions() == 1);

    // A polled request completes inside start() in either mode.
    Host::Request sync = request(other_cs);
    sync.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    sync.cmd_len = 1;
    CHECK(Host::start(sync));
    CHECK_FALSE(Host::pending());
    CHECK(other().bytes == 2);

    // And the counters say which style each requester chose.
    CHECK(Host::pumped_requests() == 1);
    CHECK(Host::polled_requests() == 1);

    // recover() drops a held transfer without touching the wire.
    CHECK_FALSE(Host::start(async));
    CHECK(Host::pending());
    Host::recover();
    CHECK_FALSE(Host::pending());
    CHECK(other().bytes == 2);
}

// =============================================================================
// the arbiter over the simulated host
// =============================================================================

namespace {

using Arb = SpiBus<Host, HostPlatform, 4>;

struct Client : Fsm<Client, SpiDone> {
    static inline EventQueue<Event, 8, HostPlatform> queue;
    static inline std::vector<uint8_t> replies;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(
            e, [](Entry) { return handled(); },
            [](SpiDone d) {
                replies.push_back(d.status);
                return handled();
            },
            [](auto) { return unhandled(); });
    }
};

using K = Tenuto<HostPlatform, Client, Arb>;

void arbiter_fresh() {
    fresh();
    HostPlatform::reset();
    Client::replies.clear();
    while (Arb::queue.pop().has_value()) {
    }
    while (Client::queue.pop().has_value()) {
    }
    K::init_all();
}

}   // namespace

TEST_CASE("the arbiter over the simulated host: a synchronous completion replies at once") {
    arbiter_fresh();
    static const uint8_t cmd[1] = {Dcs::dispon};

    Host::Request r = request(other_cs);
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.cmd_len = 1;
    r.reply = reply_to<Client, SpiDone>();
    post<Arb>(r);
    while (K::step()) {
    }

    CHECK(Client::replies == std::vector<uint8_t>{spi_ok});
    CHECK(other().bytes == 1);
    CHECK(Host::transactions() == 1);
}

TEST_CASE("the arbiter over the simulated host: a deferred completion replies when the glue posts") {
    arbiter_fresh();
    Host::completion(SimSpiCompletion::deferred);
    static const uint8_t cmd[1] = {Dcs::dispon};

    Host::Request r = request(other_cs, false);
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.cmd_len = 1;
    r.reply = reply_to<Client, SpiDone>();
    post<Arb>(r);
    while (K::step()) {
    }
    // The engine "runs on its interrupt": nothing on the wire, no reply.
    CHECK(Client::replies.empty());
    CHECK(Host::pending());
    CHECK(other().bytes == 0);

    // THE TEST IS THE APP'S ISR GLUE: finish the transfer, post what a
    // vector binding would post.
    const uint8_t status = Host::finish();
    post<Arb>(TransferDone{status});
    while (K::step()) {
    }

    CHECK(other().bytes == 1);
    CHECK(Client::replies == std::vector<uint8_t>{spi_ok});
}

// =============================================================================
// the link over the simulated host, against the simulated ILI9481
// =============================================================================

TEST_CASE("the device code comes back raw, one dummy clock late") {
    fresh();
    Link link{link_config()};
    REQUIRE(link.valid());
    wake(link);

    uint8_t raw[8] = {};
    REQUIRE(link.read(Ili9481::device_code_command, raw));
    // The link hands back what it clocked and shifts nothing: applying
    // the controller's framing is the driver's job.
    CHECK(raw[0] == 0x01);
    CHECK(raw[1] == 0x02);
    CHECK(raw[2] == 0x4A);
    CHECK(raw[3] == 0x40);
    CHECK(raw[4] == 0xFF);
    CHECK(raw[7] == 0xFF);
    CHECK(link.status() == spi_ok);
    // The read tenure's own prototype was used: its setup time.
    CHECK(Host::waited_us() == 1);
}

TEST_CASE("the single-parameter reads are aligned and the identification registers are not driven") {
    fresh();
    Link link{link_config()};
    wake(link);

    uint8_t pm[4] = {};
    REQUIRE(link.read(Dcs::rddpm, pm));
    CHECK(pm[0] == 0x1C);   // sleep out, normal mode, display on

    const uint8_t code[1] = {0x40};
    REQUIRE(link.command(Dcs::madctl, code));
    uint8_t mad[4] = {};
    REQUIRE(link.read(Dcs::rddmadctl, mad));
    CHECK(mad[0] == 0x40);

    uint8_t col[4] = {};
    REQUIRE(link.read(Dcs::rddcolmod, col));
    CHECK(col[0] == 0x66);   // eighteen bits a pixel, the reset value

    uint8_t id[6] = {};
    REQUIRE(link.read(Dcs::rddid, id));
    for (uint8_t b : id) {
        CHECK(b == 0xFF);
    }
    uint8_t st[7] = {};
    REQUIRE(link.read(Dcs::rddst, st));
    for (uint8_t b : st) {
        CHECK(b == 0xFF);
    }
}

TEST_CASE("the wake commands and a window reach the panel through the link") {
    fresh();
    Link link{link_config()};

    CHECK(panel().sleeping());
    CHECK_FALSE(panel().display_on());
    REQUIRE(link.command(Dcs::slpout, {}));
    CHECK_FALSE(panel().sleeping());
    REQUIRE(link.command(Dcs::dispon, {}));
    CHECK(panel().display_on());

    const uint8_t code[1] = {DcsAddressMode::column_order};
    REQUIRE(link.command(Dcs::madctl, code));
    CHECK(panel().madctl() == 0x40);

    set_window(link, 16, 23, 8, 11);
    // Five commands, and every one of them a select window of its own.
    CHECK(front().transactions() == 5);
    CHECK(panel().commands_taken() == 5);
    CHECK(panel().unknown_commands() == 0);
}

TEST_CASE("a memory write reads back pixel for pixel behind its dummy byte, and a continue continues") {
    fresh();
    Link link{link_config()};
    wake(link);

    constexpr uint16_t x = 16, y = 16, w = 8, h = 4;
    constexpr uint32_t n = w * h;
    constexpr uint32_t bytes = n * 3u;

    uint8_t want[bytes] = {};
    for (uint16_t r = 0; r < h; ++r) {
        for (uint16_t c = 0; c < w; ++c) {
            const std::array<uint8_t, 3> p =
                dcs_rgb666_pack(pattern(static_cast<uint16_t>(x + c), static_cast<uint16_t>(y + r)));
            const uint32_t at = (static_cast<uint32_t>(r) * w + c) * 3u;
            want[at] = p[0];
            want[at + 1u] = p[1];
            want[at + 2u] = p[2];
        }
    }

    set_window(link, x, x + w - 1u, y, y + h - 1u);
    REQUIRE(link.write(Dcs::ramwr, std::span<const uint8_t>(want, bytes)));
    CHECK(panel().pixels_written() == n);
    CHECK(panel().writes_dropped() == 0);

    // The window again, to put the read pointer back at its start, then
    // one read of the whole block: the dummy byte first, the pixels
    // after it, as stored.
    set_window(link, x, x + w - 1u, y, y + h - 1u);
    uint8_t raw[bytes + 1u] = {};
    REQUIRE(link.read(Dcs::ramrd, raw));
    CHECK(raw[0] == Ili9481::memory_read_dummy_byte);
    uint32_t bad = 0;
    uint8_t low = 0;
    for (uint32_t i = 0; i < bytes; ++i) {
        if (raw[i + 1u] != want[i]) {
            ++bad;
        }
        low = static_cast<uint8_t>(low | (raw[i + 1u] & 0x03u));
    }
    CHECK(bad == 0);
    CHECK(low == 0x00);   // the two low bits a write drops

    // A read that continues: exactly what four pixels need, then the
    // rest through 3Eh.
    set_window(link, x, x + w - 1u, y, y + h - 1u);
    uint8_t head[1u + 4u * 3u] = {};
    REQUIRE(link.read(Dcs::ramrd, head));
    uint8_t tail[1u + 4u * 3u] = {};
    REQUIRE(link.read(Dcs::ramrd_continue, tail));
    for (uint32_t i = 0; i < 12u; ++i) {
        CHECK(head[i + 1u] == want[i]);
        CHECK(tail[i + 1u] == want[12u + i]);
    }

    // Every link call is one tenure, and one core transaction.
    CHECK(front().transactions() == panel().commands_taken());
}

TEST_CASE("the BGR bit acts on the write alone, and the link carries the bytes either way") {
    fresh();
    Link link{link_config()};
    wake(link);

    const uint8_t code[1] = {DcsAddressMode::bgr};
    REQUIRE(link.command(Dcs::madctl, code));
    set_window(link, 4, 4, 4, 4);
    const uint8_t pixel[3] = {0x40, 0x80, 0xC0};
    REQUIRE(link.write(Dcs::ramwr, pixel));

    // Stored with the first and third byte exchanged...
    set_window(link, 4, 4, 4, 4);
    uint8_t raw[4] = {};
    REQUIRE(link.read(Dcs::ramrd, raw));
    CHECK(raw[0] == Ili9481::memory_read_dummy_byte);
    CHECK(raw[1] == 0xC0);
    CHECK(raw[2] == 0x80);
    CHECK(raw[3] == 0x40);
    // ... and read back AS STORED: the bit does not undo itself.
    CHECK(panel().stored(4, 4) == 0x00C08040u);
}

TEST_CASE("a prototype whose frame is not a byte is refused, and no transaction reaches the panel") {
    fresh();
    Link::Config config = link_config();
    config.read.bits = SimSpiDataSize::bits16;
    Link link{config};

    CHECK_FALSE(link.valid());
    CHECK(link.write_prototype_valid());
    CHECK_FALSE(link.read_prototype_valid());

    // Every verb refuses while the configuration is refused - the write
    // ones too, because a link with one good tenure and one bad is not
    // half usable.
    const uint8_t code[1] = {0x40};
    uint8_t in[4] = {};
    CHECK_FALSE(link.command(Dcs::madctl, code));
    CHECK_FALSE(link.write(Dcs::ramwr, code));
    CHECK_FALSE(link.read(Dcs::rddmadctl, in));
    CHECK(Host::transactions() == 0);
    CHECK(front().transactions() == 0);
    CHECK(panel().commands_taken() == 0);
}

TEST_CASE("a verb answers with the engine's status, and on a bus made of RAM it cannot fail") {
    fresh();
    Link link{link_config()};

    // There is no failure case to stage here, and that is the point: a
    // verb is `start() && status() == spi_ok`, and this world has no
    // wire to fail on and no engine to wedge - a refusal is the only
    // false it can produce (the case above). What the pair buys is the
    // engine's own code on the silicon, where a polled request over DMA
    // engines can come back spi_dma_fault inside start().
    REQUIRE(link.command(Dcs::nop, {}));
    CHECK(link.status() == spi_ok);
    CHECK(Host::status() == spi_ok);
    CHECK(Host::polled_requests() == 1);
    CHECK(Host::pumped_requests() == 0);   // the link's tenures are polled
}
