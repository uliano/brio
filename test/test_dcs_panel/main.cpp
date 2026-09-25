// Host tests for the panel driver (brio/devices/dcs_panel.hpp): the
// fourth layer of the command tier, over the three under it.
//
// THE FIXTURE IS THE WHOLE STACK, bottom up: a simulated ILI9481 core
// (brio/host/sim_dcs_panel.hpp) behind its four-wire framing adapter, on
// a simulated SPI host at the level of the Request
// (brio/host/sim_spi_host.hpp), under the serial link
// (brio/devices/dcs_link.hpp), under the driver. Nothing is mocked out
// between them: what a case asserts about a pixel, it asserts about the
// bytes the driver put on a bus.
//
// TWO MODULES, because the module's facts are what the rotation map has
// to answer: the BENCH module (its glass mirrored, its channels crossed,
// wanting the inversion) and a PLAIN one with none of that. Both cores
// answer `glass(d, line)` with what an EYE would see - the mirror, the
// colour order and the inversion already applied - so a case can say
// "the logical origin is red at the glass's top-left corner" and mean
// it, whichever module it is talking about.
//
// WHAT JUDGES WHAT. The rotation map and the window arithmetic are
// judged against the simulated memory, which is the bench's own
// measurements written down (docs/devices/ili9481.md); the primitives
// above the driver are judged against the reference renderer
// (brio/host/gfx_reference.hpp), which shares no arithmetic with them.
// Neither plane can say whether the SILICON agrees - only the bench can,
// and a bench finding corrects the simulator and never the other way
// round (docs/design/gfx.md, "The three planes of truth").
//
// Run with: ctest --preset host (or ctest --preset host -R test_dcs_panel)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include <array>
#include <span>
#include <vector>

#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "devices/dcs_panel.hpp"
#include "devices/ili9481.hpp"
#include "gfx/draw.hpp"
#include "gfx/font_5x7.hpp"
#include "gfx/surface.hpp"
#include "gfx/text.hpp"
#include "host/gfx_reference.hpp"
#include "host/sim_dcs_panel.hpp"
#include "host/sim_spi_host.hpp"

using namespace brio;

namespace {

using Host = SimSpiHost<0>;
using Link = DcsSerialLink<Host>;

using Core = SimDcsPanel<Ili9481>;
// The buffer of a framing adapter is a chip's, and this one has to hold
// a whole row of the ROTATED surface: 480 pixels of three bytes.
using Front = SimDcsSerial<Core, 2048>;
using Panel = DcsPanel<Ili9481, Link>;

/**
 * The ILI9481's every rule on a glass small enough to read WHOLE. The
 * reference renderer asks a surface for every one of its pixels, and on
 * this controller that is 153600 windows and a read apiece - an hour of
 * a simulated bus to judge arithmetic the size of the array does not
 * enter. So the judge's panel differs from the bench's in its GEOMETRY
 * and in nothing else: the same walk, the same framing, the same
 * packing, the same driver.
 */
struct SmallIli9481 : Ili9481 {
    static constexpr uint16_t columns = 64;
    static constexpr uint16_t pages = 96;
};

using SmallCore = SimDcsPanel<SmallIli9481>;
using SmallFront = SimDcsSerial<SmallCore, 512>;
using SmallPanel = DcsPanel<SmallIli9481, Link>;

/// The select lines of this imaginary board.
constexpr SimSpiPinRef bench_cs{2};
constexpr SimSpiPinRef dc_pin{3};
constexpr SimSpiPinRef plain_cs{4};
constexpr SimSpiPinRef small_cs{6};

/// The bench module and a module with none of its quirks, each stated
/// twice - once for the world (the simulator's glass) and once for the
/// driver (the facts a board file hands it).
constexpr SimDcsGlass bench_glass{.column_mirror = true, .bgr = true, .wants_inversion = true};
constexpr DcsModule bench_module{.column_mirror = true, .bgr = true, .wants_inversion = true};
constexpr SimDcsGlass plain_glass{};
constexpr DcsModule plain_module{};

/// One core of each: 460800 bytes of frame memory has no business on a
/// stack.
Core& bench_core() {
    static Core c{bench_glass};
    return c;
}
Front& bench_front() {
    static Front f{bench_core()};
    return f;
}
Core& plain_core() {
    static Core c{plain_glass};
    return c;
}
Front& plain_front() {
    static Front f{plain_core()};
    return f;
}
SmallCore& small_core() {
    static SmallCore c{bench_glass};
    return c;
}
SmallFront& small_front() {
    static SmallFront f{small_core()};
    return f;
}

/// The prototype pair an application would hand the link: the write
/// tenure and the slower read one, both on one select line.
Link::Config link_config(SimSpiPinRef cs) {
    Host::Request write{};
    write.cs = cs;
    write.dc = dc_pin;
    write.cs_setup_us = 1;
    write.clock = SimSpiClock::div4;
    write.mode = SimSpiMode::mode0;
    write.bits = SimSpiDataSize::bits8;
    Host::Request read = write;
    read.clock = SimSpiClock::div16;
    return Link::Config{write, read};
}

/// A link and a panel over it, in one object, so that the panel's
/// reference to the link cannot outlive it.
template <typename P>
struct Rig {
    Link link;
    P panel;

    Rig(SimSpiPinRef cs, DcsModule module) : link(link_config(cs)), panel(link, module) {}
};

using BenchRig = Rig<Panel>;
using SmallRig = Rig<SmallPanel>;

/// A reset line that remembers what was done to it and when.
struct RecordingPin {
    uint32_t sets = 0;
    uint32_t clears = 0;
    bool level = false;
    bool cleared_before_set = false;

    void set() {
        if (clears != 0 && sets == 0) {
            cleared_before_set = true;
        }
        level = true;
        ++sets;
    }
    void clear() {
        level = false;
        ++clears;
    }
};

/// A clock that counts milliseconds instead of spending them - which is
/// the whole reason the wake takes its delay as a callable.
struct WaitLog {
    std::vector<uint16_t> ms;

    void operator()(uint16_t n) { ms.push_back(n); }
};

/// A bus with the two panels on their select lines, nothing counted and
/// nothing remembered.
void fresh() {
    Host::release();
    Host::completion(SimSpiCompletion::immediate);
    SimSpiBench::reset();
    bench_core().hard_reset();
    bench_core().reset_counters();
    bench_front().reset_counters();
    bench_front().select(false);
    plain_core().hard_reset();
    plain_core().reset_counters();
    plain_front().reset_counters();
    plain_front().select(false);
    small_core().hard_reset();
    small_core().reset_counters();
    small_front().reset_counters();
    small_front().select(false);
    REQUIRE(Host::attach(bench_cs, bench_front()));
    REQUIRE(Host::attach(plain_cs, plain_front()));
    REQUIRE(Host::attach(small_cs, small_front()));
    Host::reset_counters();
    SimSpiBench::reset();
}

/// The wake, with a pin and a clock nobody looks at.
template <typename P>
void wake(Rig<P>& rig) {
    RecordingPin reset;
    WaitLog waits;
    REQUIRE(rig.panel.reset_and_wake(reset, waits));
}

/// A pixel's colour from its position, the pattern the bench probe
/// wrote its frame with: every byte moves with x and y, so a stream one
/// byte out of step disagrees at once. Already masked to what eighteen
/// bits keep.
constexpr uint32_t pattern(uint16_t x, uint16_t y) {
    uint32_t v = static_cast<uint32_t>(x) * 0x9E3779B1u;
    v ^= (static_cast<uint32_t>(y) + 1u) * 0x85EBCA77u;
    v ^= v >> 15;
    v *= 0x2C1B3C6Du;
    v ^= v >> 12;
    return v & 0x00FCFCFCu;
}

/// Where a colour written to the logical origin, to the far end of the
/// first ROW and to the far end of the first COLUMN must land on the
/// glass, for each rotation - the picture's three corners against the
/// glass's own four. Written as the numbers and not as a formula: a
/// table the driver's arithmetic has to meet, rather than a second copy
/// of it.
struct Spot {
    uint16_t d;
    uint16_t line;
};

struct Corners {
    Spot origin;   ///< logical (0, 0)
    Spot x_end;    ///< logical (width - 1, 0)
    Spot y_end;    ///< logical (0, height - 1)
};

constexpr uint16_t last_d = Ili9481::columns - 1u;      // 319: the glass's right
constexpr uint16_t last_line = Ili9481::pages - 1u;     // 479: the glass's bottom

constexpr Corners corners_of(DcsRotation r) {
    switch (r) {
        case DcsRotation::r0:   // the glass's own portrait
            return {{0, 0}, {last_d, 0}, {0, last_line}};
        case DcsRotation::r90:  // turned clockwise: x runs DOWN the right edge
            return {{last_d, 0}, {last_d, last_line}, {0, 0}};
        case DcsRotation::r180:
            return {{last_d, last_line}, {0, last_line}, {last_d, 0}};
        case DcsRotation::r270:  // counter-clockwise: x runs UP the left edge
            return {{0, last_line}, {0, 0}, {last_d, last_line}};
    }
    return {{0, 0}, {0, 0}, {0, 0}};
}

constexpr std::array<DcsRotation, 4> every_rotation{DcsRotation::r0, DcsRotation::r90,
                                                    DcsRotation::r180, DcsRotation::r270};

const char* rotation_name(DcsRotation r) {
    switch (r) {
        case DcsRotation::r0: return "r0";
        case DcsRotation::r90: return "r90";
        case DcsRotation::r180: return "r180";
        case DcsRotation::r270: return "r270";
    }
    return "?";
}

/// The three colours the corner cases use, each already masked to the
/// six bits a pixel keeps.
constexpr uint32_t red = 0x00FC0000u;
constexpr uint32_t green = 0x0000FC00u;
constexpr uint32_t blue = 0x000000FCu;

/// A drawing whose shapes lie in four disjoint horizontal bands, so
/// that no shape can overwrite another - which matters because a text
/// cell paints its own background and the reference renderer, which
/// only ever adds pixels, would keep what the panel erased.
struct Scene {
    Coord text_x, text_y;
    Coord x0, y0, x1, y1;
    Coord cx, cy;
    Extent cr;
    Coord rx, ry;
    Extent rw, rh, rr;
};

constexpr Scene scene_for(Extent w, Extent h) {
    const Coord band = static_cast<Coord>(h / 4);
    return Scene{
        .text_x = 2,
        .text_y = 2,
        .x0 = 1,
        .y0 = static_cast<Coord>(band + 1),
        .x1 = static_cast<Coord>(w - 2),
        .y1 = static_cast<Coord>(band + band / 2),
        .cx = static_cast<Coord>(w / 2),
        .cy = static_cast<Coord>(2 * band + band / 2),
        .cr = static_cast<Extent>(band / 2 - 2),
        .rx = 2,
        .ry = static_cast<Coord>(3 * band + 1),
        .rw = static_cast<Extent>(w - 4),
        .rh = static_cast<Extent>(band - 3),
        .rr = 4,
    };
}

template <typename S>
void draw_scene(S& s, const Scene& sc, typename S::Color fg, typename S::Color bg) {
    (void)text<Font5x7>(s, sc.text_x, sc.text_y, "brio", fg, bg);
    line(s, sc.x0, sc.y0, sc.x1, sc.y1, fg);
    circle(s, sc.cx, sc.cy, sc.cr, fg);
    fill_round_rect(s, sc.rx, sc.ry, sc.rw, sc.rh, sc.rr, fg);
}

void draw_scene(RefCanvas& ref, const Scene& sc) {
    ref.text<Font5x7>(sc.text_x, sc.text_y, "brio", 1, 0);
    ref.line(sc.x0, sc.y0, sc.x1, sc.y1, 1);
    ref.circle(sc.cx, sc.cy, sc.cr, 1);
    ref.fill_round_rect(sc.rx, sc.ry, sc.rw, sc.rh, sc.rr, 1);
}

}   // namespace

// =============================================================================
// what the driver is
// =============================================================================

TEST_CASE("the panel is a surface, and a readable one") {
    static_assert(Surface<Panel>);
    static_assert(ReadableSurface<Panel>);
    static_assert(Surface<SmallPanel>);
    static_assert(ReadableSurface<SmallPanel>);
    static_assert(DcsLink<Link>);

    // The format is the drawing world's colour and the panel's own
    // business is how much of it survives.
    static_assert(Panel::Format::bits == 24);
    static_assert(Panel::Format::max_color == 0x00FFFFFFu);
    CHECK(true);
}

TEST_CASE("the rotation map is the walk along a run, folded with the module's mirror") {
    // The eight codes, as constants a reader can check against the
    // controller's own table: no BGR bit, no display bit, and the
    // mirror touching only the rotations whose runs walk the columns.
    static_assert(dcs_rotation_madctl(DcsRotation::r0, false) == 0x00);
    static_assert(dcs_rotation_madctl(DcsRotation::r0, true) == 0x40);
    static_assert(dcs_rotation_madctl(DcsRotation::r90, false) == 0x20);
    static_assert(dcs_rotation_madctl(DcsRotation::r90, true) == 0x20);
    static_assert(dcs_rotation_madctl(DcsRotation::r180, false) == 0x40);
    static_assert(dcs_rotation_madctl(DcsRotation::r180, true) == 0x00);
    static_assert(dcs_rotation_madctl(DcsRotation::r270, false) == 0xA0);
    static_assert(dcs_rotation_madctl(DcsRotation::r270, true) == 0xA0);

    for (DcsRotation r : every_rotation) {
        const uint8_t code = dcs_rotation_madctl(r, true);
        INFO(rotation_name(r));
        CHECK((code & DcsAddressMode::bgr) == 0);
        CHECK((code & DcsAddressMode::line_order) == 0);
        CHECK((code & DcsAddressMode::horizontal_flip) == 0);
        CHECK((code & DcsAddressMode::vertical_flip) == 0);
    }

    // And the byte the panel actually writes is that map's.
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);
    for (DcsRotation r : every_rotation) {
        INFO(rotation_name(r));
        REQUIRE(rig.panel.rotation(r));
        CHECK(rig.panel.rotation() == r);
        CHECK(rig.panel.madctl() == dcs_rotation_madctl(r, true));
        CHECK(bench_core().madctl() == dcs_rotation_madctl(r, true));
    }
}

// =============================================================================
// the wake
// =============================================================================

TEST_CASE("the wake drives the reset line and spends the traits' four waits in order") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    RecordingPin reset;
    WaitLog waits;

    CHECK(bench_core().sleeping());
    CHECK_FALSE(bench_core().display_on());

    REQUIRE(rig.panel.reset_and_wake(reset, waits));

    // The line: low, then high, and left high.
    CHECK(reset.clears == 1);
    CHECK(reset.sets == 1);
    CHECK(reset.cleared_before_set);
    CHECK(reset.level);

    // The four waits, the traits' numbers in the traits' order.
    REQUIRE(waits.ms.size() == 4);
    CHECK(waits.ms[0] == Ili9481::reset_low_ms);
    CHECK(waits.ms[1] == Ili9481::reset_settle_ms);
    CHECK(waits.ms[2] == Ili9481::sleep_out_ms);
    CHECK(waits.ms[3] == Ili9481::display_on_ms);

    // And what the panel holds afterwards.
    CHECK_FALSE(bench_core().sleeping());
    CHECK(bench_core().display_on());
    CHECK(bench_core().inverted());   // this module wants it
    CHECK(bench_core().colmod() == dcs_colmod_18bpp);
    CHECK(bench_core().madctl() == dcs_rotation_madctl(DcsRotation::r0, true));
    CHECK(rig.panel.link_failures() == 0);
}

TEST_CASE("a module that does not want the inversion is woken without it") {
    fresh();
    BenchRig rig{plain_cs, plain_module};
    wake(rig);

    CHECK_FALSE(plain_core().inverted());
    CHECK(plain_core().display_on());
    CHECK(plain_core().madctl() == dcs_rotation_madctl(DcsRotation::r0, false));
}

TEST_CASE("width and height follow the rotation") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);

    REQUIRE(rig.panel.rotation(DcsRotation::r0));
    CHECK(rig.panel.width() == Ili9481::columns);
    CHECK(rig.panel.height() == Ili9481::pages);
    REQUIRE(rig.panel.rotation(DcsRotation::r180));
    CHECK(rig.panel.width() == Ili9481::columns);
    CHECK(rig.panel.height() == Ili9481::pages);

    REQUIRE(rig.panel.rotation(DcsRotation::r90));
    CHECK(rig.panel.width() == Ili9481::pages);
    CHECK(rig.panel.height() == Ili9481::columns);
    REQUIRE(rig.panel.rotation(DcsRotation::r270));
    CHECK(rig.panel.width() == Ili9481::pages);
    CHECK(rig.panel.height() == Ili9481::columns);
}

// =============================================================================
// the rotation map, on the glass
// =============================================================================

/// Three pixels through gfx's own set_pixel, then the GLASS asked where
/// they are - the module's mirror, its colour order and its inversion
/// all already applied by the verb, so the answer is what an eye would
/// see and the expectations are the same for both modules.
template <typename R>
void check_corners_on_the_glass(R& rig, Core& core) {
    for (DcsRotation r : every_rotation) {
        INFO(rotation_name(r));
        REQUIRE(rig.panel.rotation(r));
        const Extent w = rig.panel.width();
        const Extent h = rig.panel.height();

        set_pixel(rig.panel, 0, 0, red);
        set_pixel(rig.panel, static_cast<Coord>(w - 1), 0, green);
        set_pixel(rig.panel, 0, static_cast<Coord>(h - 1), blue);

        const Corners c = corners_of(r);
        CHECK(core.glass(c.origin.d, c.origin.line) == red);
        CHECK(core.glass(c.x_end.d, c.x_end.line) == green);
        CHECK(core.glass(c.y_end.d, c.y_end.line) == blue);

        // And the same three read back in LOGICAL coordinates.
        CHECK(rig.panel.get_pixel(0, 0) == red);
        CHECK(rig.panel.get_pixel(static_cast<Coord>(w - 1), 0) == green);
        CHECK(rig.panel.get_pixel(0, static_cast<Coord>(h - 1)) == blue);
    }
}

TEST_CASE("the picture's corners land on the glass's corners - the bench module") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);
    check_corners_on_the_glass(rig, bench_core());
}

TEST_CASE("the picture's corners land on the glass's corners - a plain module") {
    fresh();
    BenchRig rig{plain_cs, plain_module};
    wake(rig);
    check_corners_on_the_glass(rig, plain_core());
}

// =============================================================================
// the oracle round trip
// =============================================================================

TEST_CASE("a run written comes back in logical coordinates, under every rotation") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);

    constexpr Extent n = 24;
    for (DcsRotation r : every_rotation) {
        INFO(rotation_name(r));
        REQUIRE(rig.panel.rotation(r));

        std::array<uint32_t, n> want{};
        for (Extent i = 0; i < n; ++i) {
            want[i] = pattern(static_cast<uint16_t>(30 + i), 17);
        }
        rig.panel.write_run(30, 17, std::span<const uint32_t>(want));

        std::array<uint32_t, n> got{};
        REQUIRE(rig.panel.read_run(30, 17, std::span<uint32_t>(got)));
        CHECK(got == want);

        // A pixel of that run, one window at a time.
        CHECK(rig.panel.get_pixel(30, 17) == want[0]);
        CHECK(rig.panel.get_pixel(static_cast<Coord>(30 + n - 1), 17) == want[n - 1]);
    }
    CHECK(rig.panel.link_failures() == 0);
}

TEST_CASE("a filled rectangle reads back row for row") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);
    REQUIRE(rig.panel.rotation(DcsRotation::r270));

    constexpr Coord x = 40, y = 25;
    constexpr Extent w = 12, h = 5;
    constexpr uint32_t colour = 0x00A45C10u & 0x00FCFCFCu;
    rig.panel.fill_rect(x, y, w, h, colour);

    for (Extent row = 0; row < h; ++row) {
        INFO("row " << row);
        std::array<uint32_t, w> got{};
        REQUIRE(rig.panel.read_run(x, static_cast<Coord>(y + row), std::span<uint32_t>(got)));
        for (Extent i = 0; i < w; ++i) {
            CHECK(got[i] == colour);
        }
    }
    // The cells either side of the rectangle are untouched: the reset
    // fill is what the memory still holds there.
    CHECK(rig.panel.get_pixel(static_cast<Coord>(x - 1), y) != colour);
    CHECK(rig.panel.get_pixel(static_cast<Coord>(x + w), y) != colour);
}

TEST_CASE("a run as wide as the rotated surface - the long side - survives whole") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);
    REQUIRE(rig.panel.rotation(DcsRotation::r90));
    REQUIRE(rig.panel.width() == Ili9481::pages);

    std::vector<uint32_t> want(rig.panel.width());
    for (Extent i = 0; i < rig.panel.width(); ++i) {
        want[i] = pattern(i, 100);
    }
    rig.panel.write_run(0, 100, std::span<const uint32_t>(want));

    std::vector<uint32_t> got(rig.panel.width());
    REQUIRE(rig.panel.read_run(0, 100, std::span<uint32_t>(got)));
    CHECK(got == want);
    CHECK(bench_front().overruns() == 0);
}

// =============================================================================
// clipping
// =============================================================================

TEST_CASE("a rectangle half outside writes exactly the cells that are inside") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);
    bench_core().reset_counters();

    rig.panel.fill_rect(-5, -5, 10, 10, red);
    CHECK(bench_core().pixels_written() == 25);
    CHECK(bench_core().writes_dropped() == 0);
    CHECK(rig.panel.get_pixel(0, 0) == red);
    CHECK(rig.panel.get_pixel(4, 4) == red);
    CHECK(rig.panel.get_pixel(5, 4) != red);

    // The far edges, the other two directions.
    bench_core().reset_counters();
    const Coord right = static_cast<Coord>(rig.panel.width() - 3);
    const Coord bottom = static_cast<Coord>(rig.panel.height() - 2);
    rig.panel.fill_rect(right, bottom, 8, 8, green);
    CHECK(bench_core().pixels_written() == 3u * 2u);
}

TEST_CASE("a run that starts left of zero loses its head and keeps its tail") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);
    bench_core().reset_counters();

    const std::array<uint32_t, 5> run{pattern(0, 0), pattern(1, 0), pattern(2, 0), pattern(3, 0),
                                      pattern(4, 0)};
    rig.panel.write_run(-2, 6, std::span<const uint32_t>(run));
    CHECK(bench_core().pixels_written() == 3);

    std::array<uint32_t, 3> got{};
    REQUIRE(rig.panel.read_run(0, 6, std::span<uint32_t>(got)));
    CHECK(got[0] == run[2]);
    CHECK(got[1] == run[3]);
    CHECK(got[2] == run[4]);
}

TEST_CASE("what lies wholly outside the surface never reaches the link") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);
    Host::reset_counters();
    bench_core().reset_counters();

    rig.panel.fill_rect(static_cast<Coord>(rig.panel.width()), 0, 10, 10, red);
    rig.panel.fill_rect(0, -20, 10, 10, red);
    const std::array<uint32_t, 4> run{red, red, red, red};
    rig.panel.write_run(0, static_cast<Coord>(rig.panel.height()), std::span<const uint32_t>(run));
    rig.panel.write_run(-9, 0, std::span<const uint32_t>(run));

    CHECK(Host::transactions() == 0);
    CHECK(bench_core().commands_taken() == 0);

    // A pixel outside answers 0 and asks nobody, and a run outside is
    // not a refusal: nothing was refused.
    CHECK(rig.panel.get_pixel(-1, 0) == 0);
    CHECK(rig.panel.get_pixel(0, static_cast<Coord>(rig.panel.height())) == 0);
    std::array<uint32_t, 3> got{0xDEADu, 0xDEADu, 0xDEADu};
    CHECK(rig.panel.read_run(-100, 0, std::span<uint32_t>(got)));
    CHECK(got[0] == 0);
    CHECK(got[2] == 0);
    CHECK(Host::transactions() == 0);
    CHECK(rig.panel.link_failures() == 0);
}

TEST_CASE("a run partly outside comes back clipped, the outside pixels zero") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);

    const std::array<uint32_t, 4> run{pattern(7, 7), pattern(8, 7), pattern(9, 7), pattern(10, 7)};
    rig.panel.write_run(-1, 7, std::span<const uint32_t>(run));

    std::array<uint32_t, 4> got{};
    REQUIRE(rig.panel.read_run(-1, 7, std::span<uint32_t>(got)));
    CHECK(got[0] == 0);   // outside: no colour to invent
    CHECK(got[1] == run[1]);
    CHECK(got[2] == run[2]);
    CHECK(got[3] == run[3]);
}

// =============================================================================
// the reference judge
// =============================================================================

/// The primitives drawn through the panel and through the renderer that
/// computes each shape from its definition, compared pixel for pixel.
/// A lit pixel is any non-zero colour, so the ground is painted black
/// first - which is also what a write-only surface has instead of an
/// erase.
void judge_against_the_reference(SmallRig& rig, DcsRotation r) {
    INFO(rotation_name(r));
    REQUIRE(rig.panel.rotation(r));
    const Extent w = rig.panel.width();
    const Extent h = rig.panel.height();
    const Scene sc = scene_for(w, h);

    rig.panel.fill_rect(0, 0, w, h, 0);
    draw_scene(rig.panel, sc, 0x00FCFCFCu, 0u);

    RefCanvas ref(w, h);
    draw_scene(ref, sc);

    const GfxDiff d = compare(rig.panel, ref);
    INFO(d.map);
    REQUIRE(d.agree());
}

TEST_CASE("the reference renderer and the panel agree, upright and turned") {
    fresh();
    SmallRig rig{small_cs, bench_module};
    wake(rig);

    judge_against_the_reference(rig, DcsRotation::r0);
    judge_against_the_reference(rig, DcsRotation::r90);
    CHECK(rig.panel.link_failures() == 0);
}

// =============================================================================
// what an update costs
// =============================================================================

TEST_CASE("the verbs cost the transactions the design says and no more") {
    fresh();
    BenchRig rig{bench_cs, bench_module};
    wake(rig);

    // A rectangle: CASET, PASET, RAMWR, then a write memory continue
    // per further row - h + 2, whatever its width.
    for (Extent h = 1; h <= 4; ++h) {
        INFO("h = " << h);
        Host::reset_counters();
        rig.panel.fill_rect(10, 10, 9, h, red);
        CHECK(Host::transactions() == h + 2u);
    }

    // A run: one window, one RAMWR.
    Host::reset_counters();
    const std::array<uint32_t, 6> run{red, green, blue, red, green, blue};
    rig.panel.write_run(4, 4, std::span<const uint32_t>(run));
    CHECK(Host::transactions() == 3);

    // A read, and a pixel: one window, one RAMRD.
    Host::reset_counters();
    std::array<uint32_t, 6> got{};
    REQUIRE(rig.panel.read_run(4, 4, std::span<uint32_t>(got)));
    CHECK(Host::transactions() == 3);
    CHECK(got[0] == red);

    Host::reset_counters();
    CHECK(rig.panel.get_pixel(5, 4) == green);
    CHECK(Host::transactions() == 3);
}

// =============================================================================
// a link that refuses
// =============================================================================

TEST_CASE("a refused link counts one refusal per verb and lands nothing") {
    fresh();
    // A prototype whose frame is two bytes: the link refuses at
    // construction and every verb after it.
    Link::Config config = link_config(bench_cs);
    config.read.bits = SimSpiDataSize::bits16;
    Link link{config};
    REQUIRE_FALSE(link.valid());
    Panel panel{link, bench_module};

    RecordingPin reset;
    WaitLog waits;
    CHECK_FALSE(panel.reset_and_wake(reset, waits));
    // The pin was still driven and the waits still spent: they are not
    // the link's to refuse.
    CHECK(reset.clears == 1);
    CHECK(waits.ms.size() == 4);
    CHECK(panel.link_failures() == 5);   // SLPOUT, DISPON, INVON, COLMOD, MADCTL

    panel.reset_counters();
    CHECK(panel.link_failures() == 0);

    // Each Surface verb stops at its first refusal - a window half
    // written is not a window.
    panel.fill_rect(0, 0, 4, 4, red);
    CHECK(panel.link_failures() == 1);
    const std::array<uint32_t, 4> run{red, green, blue, red};
    panel.write_run(0, 0, std::span<const uint32_t>(run));
    CHECK(panel.link_failures() == 2);
    std::array<uint32_t, 4> got{};
    CHECK_FALSE(panel.read_run(0, 0, std::span<uint32_t>(got)));
    CHECK(panel.link_failures() == 3);
    CHECK(panel.get_pixel(0, 0) == 0);
    CHECK(panel.link_failures() == 4);
    CHECK_FALSE(panel.rotation(DcsRotation::r90));
    CHECK(panel.link_failures() == 5);

    // And nothing of it reached the bus, let alone the panel.
    CHECK(Host::transactions() == 0);
    CHECK(bench_core().commands_taken() == 0);
    CHECK(bench_core().pixels_written() == 0);
}
