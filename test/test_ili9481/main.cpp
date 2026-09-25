// Host tests for the simulated ILI9481: the panel core
// (brio/host/sim_dcs_panel.hpp) over the controller's traits
// (brio/devices/ili9481.hpp), with the four-wire serial adapter in
// front of it.
//
// THIS SUITE IS THE BENCH'S OWN LIST, TRANSCRIBED. Every case below is
// one the bench ran against the silicon, with the silicon's own numbers
// as the expected values: the device code behind its dummy clock, the
// memory read behind its dummy byte, the read that continues where the
// clocks stopped, a whole frame written and read back, the eight scan
// orders with the physical home of three logical pixels under each, and
// where a window lands once B5 has exchanged the axes. What it proves
// is that the SIMULATOR says what the panel said - which is the only
// thing a simulator can be right about, and the reason a bench finding
// corrects this file and never the other way round (design/gfx.md, "The
// three planes of truth").
//
// Run with: ctest --preset host (or ctest --preset host -R test_ili9481)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include <array>
#include <optional>
#include <span>

#include "devices/dcs.hpp"
#include "devices/ili9481.hpp"
#include "host/sim_dcs_panel.hpp"

using namespace brio;

namespace {

using Panel = SimDcsPanel<Ili9481>;
using Link = SimDcsSerial<Panel>;

constexpr uint16_t columns = Ili9481::columns;
constexpr uint16_t pages = Ili9481::pages;

/// The module the bench carried: its glass shows memory column 319 at
/// the left, and it needs the inversion.
constexpr SimDcsGlass bench_module{.column_mirror = true, .wants_inversion = true};

/// One panel and one link for the whole suite: the frame memory is
/// 460800 bytes and has no business on a stack.
Panel& panel() {
    static Panel p{bench_module};
    return p;
}

Link& link() {
    static Link l{panel()};
    return l;
}

void fresh() {
    panel().hard_reset();
    panel().reset_counters();
    link().reset_counters();
}

// ---- the core's seam, directly --------------------------------------------------

void command(uint8_t cmd, std::span<const uint8_t> in = {}) {
    panel().transaction(cmd, in, {});
}

void set_madctl(uint8_t code) {
    const uint8_t p[1] = {code};
    command(Dcs::madctl, p);
}

void set_window(uint16_t cs, uint16_t ce, uint16_t ps, uint16_t pe) {
    const std::array<uint8_t, 4> c = dcs_window(cs, ce);
    const std::array<uint8_t, 4> p = dcs_window(ps, pe);
    command(Dcs::caset, c);
    command(Dcs::paset, p);
}

void read(uint8_t cmd, std::span<uint8_t> out) {
    panel().transaction(cmd, {}, out);
}

// ---- the same thing over the wire -----------------------------------------------

/// A write transaction: the command with D/C low, its bytes with D/C
/// high, then the select rises.
void wire_write(uint8_t cmd, std::span<const uint8_t> data = {}) {
    link().select(true);
    link().dc(false);
    (void)link().byte(cmd);
    link().dc(true);
    for (uint8_t b : data) {
        (void)link().byte(b);
    }
    link().select(false);
}

/// A read transaction: `raw` takes the bytes MISO carried, dummies and
/// all - what a host's receive buffer holds and what the bench printed.
void wire_read(uint8_t cmd, std::span<uint8_t> raw) {
    link().select(true);
    link().dc(false);
    (void)link().byte(cmd);
    link().dc(true);
    for (uint8_t& b : raw) {
        b = link().byte(0xFF);
    }
    link().select(false);
}

/// The wake the bench proved, with the waits left out: there is no time
/// in this world.
void wake() {
    wire_write(Dcs::slpout);
    wire_write(Dcs::dispon);
    wire_write(Dcs::invon);
}

// ---- the probe's own pattern, transcribed ---------------------------------------

/// A pixel's colour from its position: every byte's six most
/// significant bits move with x and y, so a stream one byte out of step
/// disagrees at once. 0x00RRGGBB, already masked to what 18 bits keep.
constexpr uint32_t pattern(uint16_t x, uint16_t y, uint32_t salt = 0) {
    uint32_t v = static_cast<uint32_t>(x) * 0x9E3779B1u;
    v ^= (static_cast<uint32_t>(y) + 1u) * 0x85EBCA77u;
    v ^= salt;
    v ^= v >> 15;
    v *= 0x2C1B3C6Du;
    v ^= v >> 12;
    return v & 0x00FCFCFCu;
}

void put_pixel(uint8_t* row, uint16_t i, uint32_t colour) {
    const std::array<uint8_t, 3> p = dcs_rgb666_pack(colour);
    row[i * 3u] = p[0];
    row[i * 3u + 1u] = p[1];
    row[i * 3u + 2u] = p[2];
}

/// How a raw read stream lines up: the byte offset of the value's first
/// byte and the bit shift of every byte - the probe's own search space.
struct ReadFormat {
    uint8_t offset = 1;
    uint8_t shift = 0;
};

/// The stream as a format says: byte `offset + i`, shifted left by
/// `shift` with the next byte's top bits pulled in.
bool realign(const uint8_t* raw, uint32_t n_raw, uint8_t* out, uint32_t n, ReadFormat f) {
    const uint32_t need = f.offset + n + (f.shift != 0u ? 1u : 0u);
    if (need > n_raw) {
        return false;
    }
    for (uint32_t i = 0; i < n; ++i) {
        uint8_t b = raw[f.offset + i];
        if (f.shift != 0u) {
            b = static_cast<uint8_t>((b << f.shift) | (raw[f.offset + i + 1u] >> (8u - f.shift)));
        }
        out[i] = b;
    }
    return true;
}

uint32_t mismatches(const uint8_t* want, const uint8_t* got, uint32_t n, uint32_t& first) {
    uint32_t bad = 0;
    for (uint32_t i = 0; i < n; ++i) {
        if (want[i] != got[i]) {
            if (bad == 0u) {
                first = i;
            }
            ++bad;
        }
    }
    return bad;
}

constexpr uint32_t row_bytes = columns * 3u;
uint8_t row_buf[row_bytes];
uint8_t got_buf[row_bytes];

}   // namespace

// =============================================================================
// a - the controller answers
// =============================================================================

TEST_CASE("a: the read registers and the device code, over the wire") {
    fresh();
    wake();

    // THE DEVICE CODE comes one dummy CLOCK late: the whole stream is a
    // bit behind, which is how 02 04 94 81 FF reaches a host as
    // 01 02 4A 40 FF.
    uint8_t raw[8] = {};
    wire_read(Ili9481::device_code_command, raw);
    CHECK(raw[0] == 0x01);
    CHECK(raw[1] == 0x02);
    CHECK(raw[2] == 0x4A);
    CHECK(raw[3] == 0x40);
    CHECK(raw[4] == 0xFF);
    CHECK(raw[5] == 0xFF);
    CHECK(raw[6] == 0xFF);
    CHECK(raw[7] == 0xFF);

    // And the value behind it, found by the probe's own search: two
    // bytes of it at offset 2 with a shift of one bit.
    uint8_t value[4] = {};
    REQUIRE(realign(raw, 8, value, 4, ReadFormat{2, 1}));
    CHECK(value[0] == 0x94);
    CHECK(value[1] == 0x81);

    // A SINGLE-parameter read has no dummy at all: the value is the
    // first byte clocked.
    uint8_t pm[4] = {};
    wire_read(Dcs::rddpm, pm);
    CHECK(pm[0] == 0x1C);   // sleep out, normal mode, display on, D1 and D0 zero

    set_madctl(0x48);
    uint8_t mad[4] = {};
    wire_read(Dcs::rddmadctl, mad);
    CHECK(mad[0] == 0x48);
    set_madctl(Ili9481::reset_madctl);
    wire_read(Dcs::rddmadctl, mad);
    CHECK(mad[0] == 0x00);

    uint8_t col[4] = {};
    wire_read(Dcs::rddcolmod, col);
    CHECK(col[0] == 0x66);   // eighteen bits a pixel, the reset value

    // THIS MODULE drives neither identification register: all ones is
    // the pull-up and not an answer.
    uint8_t id[6] = {};
    wire_read(Dcs::rddid, id);
    for (uint8_t b : id) {
        CHECK(b == 0xFF);
    }
    uint8_t st[7] = {};
    wire_read(Dcs::rddst, st);
    for (uint8_t b : st) {
        CHECK(b == 0xFF);
    }
}

// =============================================================================
// b - the read format
// =============================================================================

TEST_CASE("b: a block written, its memory read back raw, the alignment found") {
    fresh();
    wake();

    constexpr uint16_t x = 16, y = 16, w = 8, h = 4;
    constexpr uint32_t n = w * h * 3u;

    set_window(x, static_cast<uint16_t>(x + w - 1u), y, static_cast<uint16_t>(y + h - 1u));
    uint8_t want[n] = {};
    for (uint16_t r = 0; r < h; ++r) {
        for (uint16_t c = 0; c < w; ++c) {
            put_pixel(want + r * w * 3u, c, pattern(static_cast<uint16_t>(x + c),
                                                    static_cast<uint16_t>(y + r)));
        }
    }
    wire_write(Dcs::ramwr, std::span<const uint8_t>(want, n));
    CHECK(panel().pixels_written() == w * h);
    CHECK(panel().writes_dropped() == 0);

    // One memory read over the whole block: the window spans the rows
    // and the address counter wraps at the window's edge by itself.
    set_window(x, static_cast<uint16_t>(x + w - 1u), y, static_cast<uint16_t>(y + h - 1u));
    uint8_t raw[n + 4u] = {};
    wire_read(Dcs::ramrd, raw);
    CHECK(raw[0] == 0x80);   // the one dummy byte, and what it reads

    // The probe's search: which (offset, shift) makes the stream the
    // block. One dummy byte, no shift.
    std::optional<ReadFormat> found;
    for (uint8_t shift = 0; shift < 8u && !found; ++shift) {
        for (uint8_t offset = 0; offset < 4u; ++offset) {
            uint8_t out[n] = {};
            if (!realign(raw, n + 4u, out, n, ReadFormat{offset, shift})) {
                continue;
            }
            uint32_t first = 0;
            if (mismatches(want, out, n, first) == 0u) {
                found = ReadFormat{offset, shift};
                break;
            }
        }
    }
    REQUIRE(found.has_value());
    CHECK(found->offset == 1);
    CHECK(found->shift == 0);

    // The two low bits of every byte the panel returns are zero.
    uint8_t low = 0;
    for (uint32_t i = 1; i < n + 1u; ++i) {
        low = static_cast<uint8_t>(low | (raw[i] & 0x03u));
    }
    CHECK(low == 0x00);
}

// =============================================================================
// c - the read that continues
// =============================================================================

TEST_CASE("c: read memory continue picks up where the clocks stopped") {
    fresh();
    wake();

    constexpr uint16_t x = 32, y = 16, w = 24;
    constexpr uint32_t eight = 8u * 3u;

    set_window(x, static_cast<uint16_t>(x + w - 1u), y, y);
    uint8_t want[w * 3u] = {};
    for (uint16_t c = 0; c < w; ++c) {
        put_pixel(want, c, pattern(static_cast<uint16_t>(x + c), y));
    }
    wire_write(Dcs::ramwr, std::span<const uint8_t>(want, w * 3u));

    // EXACTLY what eight pixels need: the dummy byte and twenty-four
    // more. A spare byte clocked would move the read pointer past a
    // pixel, which is the next part of this case.
    set_window(x, static_cast<uint16_t>(x + w - 1u), y, y);
    uint8_t raw[eight + 8u] = {};
    wire_read(Dcs::ramrd, std::span<uint8_t>(raw, eight + 1u));
    uint32_t first = 0;
    CHECK(mismatches(want, raw + 1, eight, first) == 0u);
    CHECK(panel().read_pointer() == eight);

    wire_read(Dcs::ramrd_continue, std::span<uint8_t>(raw, eight + 1u));
    CHECK(raw[0] == 0x80);   // its own dummy byte
    CHECK(mismatches(want + eight, raw + 1, eight, first) == 0u);
    CHECK(panel().read_pointer() == 2u * eight);

    // THE POINTER COUNTS THE BYTES CLOCKED. Four spare bytes - a whole
    // pixel and one byte more - and the next read starts at pixel 9:
    // the pixel cut short is read again from its first byte.
    set_window(x, static_cast<uint16_t>(x + w - 1u), y, y);
    wire_read(Dcs::ramrd, std::span<uint8_t>(raw, eight + 1u + 4u));
    CHECK(panel().read_pointer() == eight + 4u);
    wire_read(Dcs::ramrd_continue, std::span<uint8_t>(raw, eight + 1u));
    CHECK(mismatches(want + 9u * 3u, raw + 1, eight, first) == 0u);
}

// =============================================================================
// d - the whole frame
// =============================================================================

TEST_CASE("d: the whole frame written and read back byte for byte") {
    fresh();
    wake();
    set_madctl(Ili9481::reset_madctl);

    set_window(0, static_cast<uint16_t>(columns - 1u), 0, static_cast<uint16_t>(pages - 1u));
    for (uint16_t y = 0; y < pages; ++y) {
        for (uint16_t x = 0; x < columns; ++x) {
            put_pixel(row_buf, x, pattern(x, y));
        }
        command(y == 0u ? Dcs::ramwr : Dcs::ramwr_continue,
                std::span<const uint8_t>(row_buf, row_bytes));
    }
    CHECK(panel().pixels_written() == static_cast<uint32_t>(columns) * pages);
    CHECK(panel().writes_dropped() == 0);

    uint32_t bad = 0;
    for (uint16_t y = 0; y < pages; ++y) {
        for (uint16_t x = 0; x < columns; ++x) {
            put_pixel(row_buf, x, pattern(x, y));
        }
        set_window(0, static_cast<uint16_t>(columns - 1u), y, y);
        read(Dcs::ramrd, std::span<uint8_t>(got_buf, row_bytes));
        uint32_t first = 0;
        bad += mismatches(row_buf, got_buf, row_bytes, first);
    }
    CHECK(bad == 0u);
    CHECK(panel().bytes_read() == static_cast<uint32_t>(row_bytes) * pages);
}

// =============================================================================
// g - the eight scan orders, and the BGR bit
// =============================================================================

namespace {

struct Cell {
    uint16_t column;
    uint16_t page;
};

/// The physical home of logical (0,0), (1,0) and (0,1) of a window four
/// wide and two high at the origin - the bench's own table.
struct ScanOrder {
    uint8_t code;
    Cell home[3];
};

constexpr ScanOrder scan_orders[] = {
    {0x00, {{0, 0}, {1, 0}, {0, 1}}},
    {0x40, {{3, 0}, {2, 0}, {3, 1}}},
    {0x80, {{0, 1}, {1, 1}, {0, 0}}},
    {0xC0, {{3, 1}, {2, 1}, {3, 0}}},
    {0x20, {{0, 0}, {0, 1}, {1, 0}}},
    {0x60, {{1, 0}, {1, 1}, {0, 0}}},
    {0xA0, {{0, 3}, {0, 2}, {1, 3}}},
    {0xE0, {{1, 3}, {1, 2}, {0, 3}}},
};

/// The logical index of (0,0), (1,0) and (0,1) in a block four wide.
constexpr uint8_t logical_index[3] = {0, 1, 4};

}   // namespace

TEST_CASE("g: the walk bits reorder the counter inside the window and never move the origin") {
    for (const ScanOrder& order : scan_orders) {
        fresh();
        wake();
        set_madctl(order.code);
        set_window(0, 3, 0, 1);

        uint8_t block[8u * 3u] = {};
        for (uint16_t i = 0; i < 8u; ++i) {
            put_pixel(block, i, pattern(i, 0, 0x8000u + order.code));
        }
        command(Dcs::ramwr, block);
        CHECK(panel().pixels_written() == 8u);
        CHECK(panel().writes_dropped() == 0u);

        // Where the three logical pixels landed, located in the memory
        // the way the bench located them through its corner map.
        for (uint8_t k = 0; k < 3u; ++k) {
            const uint32_t want = pattern(logical_index[k], 0, 0x8000u + order.code);
            const Cell& home = order.home[k];
            CHECK(panel().stored(home.column, home.page) == want);
        }

        // And the read under the same code gives the block back, byte
        // for byte: the write's mapping and the read's are the same.
        set_window(0, 3, 0, 1);
        uint8_t back[8u * 3u] = {};
        read(Dcs::ramrd, back);
        uint32_t first = 0;
        CHECK(mismatches(block, back, sizeof(block), first) == 0u);
    }
}

TEST_CASE("g: the BGR bit acts on writes alone") {
    fresh();
    wake();
    set_madctl(DcsAddressMode::bgr);
    set_window(0, 3, 0, 1);

    uint8_t block[8u * 3u] = {};
    for (uint16_t i = 0; i < 8u; ++i) {
        put_pixel(block, i, pattern(i, 0, 0x8008u));
    }
    command(Dcs::ramwr, block);

    // The read finds the pixel where code 00 does: this bit is not part
    // of the walk.
    const uint32_t written = pattern(0, 0, 0x8008u);
    const uint32_t swapped = ((written & 0xFFu) << 16) | (written & 0xFF00u) | ((written >> 16) & 0xFFu);
    CHECK(panel().stored(0, 0) == swapped);
    CHECK(panel().stored(1, 0) != 0u);

    // ... and a pixel is read back AS STORED, so the round trip under
    // this bit is the exchange and not the identity.
    set_window(0, 3, 0, 1);
    uint8_t back[8u * 3u] = {};
    read(Dcs::ramrd, back);
    CHECK(back[0] == block[2]);
    CHECK(back[1] == block[1]);
    CHECK(back[2] == block[0]);
}

// =============================================================================
// x - where a window lands
// =============================================================================

namespace {

struct WindowCase {
    uint8_t code;
    uint16_t cs, ce, ps, pe;
    uint32_t colour;
    const char* name;
};

constexpr WindowCase window_cases[] = {
    {0x00, 200, 203, 300, 300, 0x00FCFCFCu, "base, CASET 200..203 PASET 300"},
    {0x20, 100, 103, 150, 150, 0x00FC0000u, "B5, CASET 100..103 PASET 150"},
    {0x20, 340, 343, 150, 150, 0x0000FC00u, "B5, CASET 340..343 PASET 150"},
    {0x20, 150, 150, 400, 403, 0x000000FCu, "B5, CASET 150 PASET 400..403"},
    {0x20, 150, 150, 250, 253, 0x00FCFC00u, "B5, CASET 150 PASET 250..253"},
    {0x60, 340, 343, 160, 160, 0x00FC00FCu, "B5+B6, CASET 340..343 PASET 160"},
    {0x40, 100, 103, 340, 340, 0x0000FCFCu, "B6, CASET 100..103 PASET 340"},
    {0xA0, 300, 303, 170, 170, 0x008080FCu, "B5+B7, CASET 300..303 PASET 170"},
};

/// Where one colour was found in the frame.
struct Found {
    uint32_t count = 0;
    uint16_t column_lo = 0xFFFFu;
    uint16_t column_hi = 0;
    uint16_t page_lo = 0xFFFFu;
    uint16_t page_hi = 0;
};

void blank_frame() {
    set_madctl(0x00);
    set_window(0, static_cast<uint16_t>(columns - 1u), 0, static_cast<uint16_t>(pages - 1u));
    for (uint16_t x = 0; x < columns; ++x) {
        put_pixel(row_buf, x, 0);
    }
    for (uint16_t y = 0; y < pages; ++y) {
        command(y == 0u ? Dcs::ramwr : Dcs::ramwr_continue,
                std::span<const uint8_t>(row_buf, row_bytes));
    }
}

}   // namespace

TEST_CASE("x: where a window lands once B5 has exchanged the axes") {
    fresh();
    wake();
    blank_frame();

    for (const WindowCase& c : window_cases) {
        set_madctl(c.code);
        set_window(c.cs, c.ce, c.ps, c.pe);
        uint8_t four[4u * 3u] = {};
        for (uint16_t i = 0; i < 4u; ++i) {
            put_pixel(four, i, c.colour);
        }
        command(Dcs::ramwr, four);
    }

    // Two full runs of one column each: does the walk reach page 479?
    constexpr uint32_t column_colour[2] = {0x00FC4040u, 0x0040FC40u};
    constexpr uint8_t column_code[2] = {0x20, 0x60};
    constexpr uint16_t column_at[2] = {40, 60};
    static uint8_t full[pages * 3u];
    for (uint8_t k = 0; k < 2u; ++k) {
        set_madctl(column_code[k]);
        set_window(0, static_cast<uint16_t>(pages - 1u), column_at[k], column_at[k]);
        for (uint16_t i = 0; i < pages; ++i) {
            put_pixel(full, i, column_colour[k]);
        }
        command(Dcs::ramwr, full);
    }

    // One scan of the frame, and every lit pixel accounted for.
    Found found[8];
    Found column_found[2];
    uint32_t stray = 0;
    for (uint16_t page = 0; page < pages; ++page) {
        for (uint16_t column = 0; column < columns; ++column) {
            const uint32_t v = panel().stored(column, page);
            if (v == 0u) {
                continue;
            }
            Found* slot = nullptr;
            for (uint8_t i = 0; i < 8u; ++i) {
                if (window_cases[i].colour == v) {
                    slot = &found[i];
                }
            }
            for (uint8_t k = 0; k < 2u; ++k) {
                if (column_colour[k] == v) {
                    slot = &column_found[k];
                }
            }
            if (slot == nullptr) {
                ++stray;
                continue;
            }
            ++slot->count;
            slot->column_lo = column < slot->column_lo ? column : slot->column_lo;
            slot->column_hi = column > slot->column_hi ? column : slot->column_hi;
            slot->page_lo = page < slot->page_lo ? page : slot->page_lo;
            slot->page_hi = page > slot->page_hi ? page : slot->page_hi;
        }
    }
    CHECK(stray == 0u);

    // Under the base code, CASET is the columns and PASET the page.
    CHECK(found[0].count == 4u);
    CHECK(found[0].column_lo == 200);
    CHECK(found[0].column_hi == 203);
    CHECK(found[0].page_lo == 300);
    CHECK(found[0].page_hi == 300);

    // Under B5, CASET addresses the PAGES and PASET the columns.
    CHECK(found[1].count == 4u);
    CHECK(found[1].page_lo == 100);
    CHECK(found[1].page_hi == 103);
    CHECK(found[1].column_lo == 150);
    CHECK(found[1].column_hi == 150);

    CHECK(found[2].count == 4u);
    CHECK(found[2].page_lo == 340);
    CHECK(found[2].page_hi == 343);
    CHECK(found[2].column_lo == 150);
    CHECK(found[2].column_hi == 150);

    // THE DEGENERATE CASE: a PASET past 319 under B5 addresses columns
    // that are not there. The bench saw at most one pixel in the frame;
    // this model writes none.
    CHECK(found[3].count <= 1u);

    CHECK(found[4].count == 4u);
    CHECK(found[4].page_lo == 150);
    CHECK(found[4].page_hi == 150);
    CHECK(found[4].column_lo == 250);
    CHECK(found[4].column_hi == 253);

    // B6 beside B5 reverses the walk and moves nothing.
    CHECK(found[5].count == 4u);
    CHECK(found[5].page_lo == 340);
    CHECK(found[5].page_hi == 343);
    CHECK(found[5].column_lo == 160);
    CHECK(found[5].column_hi == 160);

    CHECK(found[6].count == 4u);
    CHECK(found[6].column_lo == 100);
    CHECK(found[6].column_hi == 103);
    CHECK(found[6].page_lo == 340);
    CHECK(found[6].page_hi == 340);

    CHECK(found[7].count == 4u);
    CHECK(found[7].page_lo == 300);
    CHECK(found[7].page_hi == 303);
    CHECK(found[7].column_lo == 170);
    CHECK(found[7].column_hi == 170);

    // A full column under B5, and under B5 with B6: both reach page 479.
    for (uint8_t k = 0; k < 2u; ++k) {
        CHECK(column_found[k].count == pages);
        CHECK(column_found[k].page_lo == 0);
        CHECK(column_found[k].page_hi == pages - 1u);
        CHECK(column_found[k].column_lo == column_at[k]);
        CHECK(column_found[k].column_hi == column_at[k]);
    }
}

// =============================================================================
// the two resets
// =============================================================================

TEST_CASE("reset: the hardware line clears the memory, the command does not") {
    fresh();
    wake();
    set_madctl(0x60);
    set_window(0, 3, 0, 1);
    uint8_t block[8u * 3u] = {};
    for (uint16_t i = 0; i < 8u; ++i) {
        put_pixel(block, i, pattern(i, 0, 0x4242u));
    }
    command(Dcs::ramwr, block);

    // SWRESET: the registers go back, the memory stays.
    const uint32_t kept = panel().stored(1, 0);
    command(Dcs::swreset);
    CHECK(panel().stored(1, 0) == kept);
    CHECK(panel().madctl() == 0x00);
    CHECK(panel().colmod() == 0x66);
    CHECK(panel().sleeping());
    CHECK_FALSE(panel().display_on());

    uint8_t pm[1] = {};
    read(Dcs::rddpm, pm);
    CHECK((pm[0] & 0x10u) == 0u);   // sleep in
    CHECK((pm[0] & 0x04u) == 0u);   // display off
    uint8_t col[1] = {};
    read(Dcs::rddcolmod, col);
    CHECK(col[0] == 0x66);

    // The reset LINE: the memory comes back as alternate pages, the even
    // ones the first value.
    panel().hard_reset();
    CHECK(panel().stored(0, 0) == 0x00545454u);
    CHECK(panel().stored(319, 0) == 0x00545454u);
    CHECK(panel().stored(0, 1) == 0x00A8A8A8u);
    CHECK(panel().stored(160, 478) == 0x00545454u);
    CHECK(panel().stored(160, 479) == 0x00A8A8A8u);
    read(Dcs::rddpm, pm);
    CHECK((pm[0] & 0x10u) == 0u);
    CHECK((pm[0] & 0x04u) == 0u);
}

// =============================================================================
// the state commands, and what the core does with a command it has not met
// =============================================================================

TEST_CASE("state: the four pairs move the bits RDDPM reports") {
    fresh();
    uint8_t pm[1] = {};

    read(Dcs::rddpm, pm);
    CHECK(pm[0] == 0x08);   // asleep, display off, normal mode

    command(Dcs::slpout);
    command(Dcs::dispon);
    read(Dcs::rddpm, pm);
    CHECK(pm[0] == 0x1C);

    command(Dcs::dispoff);
    read(Dcs::rddpm, pm);
    CHECK(pm[0] == 0x18);
    command(Dcs::slpin);
    read(Dcs::rddpm, pm);
    CHECK(pm[0] == 0x08);

    command(Dcs::noron);
    CHECK(panel().normal_mode());
    command(Dcs::invon);
    CHECK(panel().inverted());
    command(Dcs::invoff);
    CHECK_FALSE(panel().inverted());
    CHECK(panel().unknown_commands() == 0u);
}

TEST_CASE("state: a command the core has not met is counted and otherwise ignored") {
    fresh();
    const uint32_t before = panel().commands_taken();

    // Gamma, the partial area and the brightness group are vocabulary
    // this core does not model: what a model built from measurements
    // says about them is nothing, and it says so by counting.
    const uint8_t one[1] = {0x42};
    command(Dcs::gamset, one);
    command(Dcs::ptlon);
    command(Dcs::wrdisbv, one);
    CHECK(panel().unknown_commands() == 3u);
    CHECK(panel().commands_taken() == before + 3u);

    // Nothing of the state moved with them.
    CHECK(panel().madctl() == Ili9481::reset_madctl);
    CHECK(panel().colmod() == Ili9481::reset_colmod);
    CHECK(panel().sleeping());

    // And a read of one answers the idle level, not a value.
    uint8_t out[3] = {};
    read(Dcs::rddisbv, out);
    CHECK(out[0] == 0xFF);
    CHECK(out[2] == 0xFF);
}

// =============================================================================
// the glass
// =============================================================================

TEST_CASE("glass: the module's mirror, the vertical flip and the inversion") {
    fresh();
    wake();   // the inversion is on, which is what this module wants
    set_madctl(0x00);
    set_window(0, 0, 0, 0);
    constexpr uint32_t colour = 0x00FC8004u;
    const std::array<uint8_t, 3> p = dcs_rgb666_pack(colour);
    command(Dcs::ramwr, p);
    REQUIRE(panel().stored(0, 0) == colour);

    // The glass shows memory column 319 at its LEFT, so memory (0,0) is
    // display column 319 of line 0.
    CHECK(panel().glass(319, 0) == colour);
    CHECK(panel().glass(0, 0) != colour);

    // The inversion off shows the complement of every kept bit.
    command(Dcs::invoff);
    CHECK(panel().glass(319, 0) == ((~colour) & 0x00FCFCFCu));
    command(Dcs::invon);
    CHECK(panel().glass(319, 0) == colour);

    // B0 mirrors the glass top to bottom.
    set_madctl(DcsAddressMode::vertical_flip);
    CHECK(panel().glass(319, pages - 1u) == colour);
    CHECK(panel().glass(319, 0) != colour);

    // B1 and B4 do NOTHING on this module: they are kept in the state
    // and applied to nothing.
    set_madctl(DcsAddressMode::horizontal_flip);
    CHECK(panel().madctl() == DcsAddressMode::horizontal_flip);
    CHECK(panel().glass(319, 0) == colour);
    set_madctl(DcsAddressMode::line_order);
    CHECK(panel().glass(319, 0) == colour);
    set_madctl(static_cast<uint8_t>(DcsAddressMode::line_order | DcsAddressMode::horizontal_flip));
    CHECK(panel().glass(319, 0) == colour);

    // Off the glass entirely.
    CHECK(panel().glass(columns, 0) == 0u);
    CHECK(panel().glass(0, pages) == 0u);
}

// =============================================================================
// the framing adapter
// =============================================================================

TEST_CASE("adapter: a command mid-transaction closes the one standing") {
    fresh();
    link().reset_counters();

    // Two commands inside ONE select: the panel ends a command when the
    // host sends another.
    link().select(true);
    link().dc(false);
    (void)link().byte(Dcs::madctl);
    link().dc(true);
    (void)link().byte(0x60);
    link().dc(false);
    (void)link().byte(Dcs::dispon);
    link().select(false);

    CHECK(panel().madctl() == 0x60);
    CHECK(panel().display_on());
    CHECK(link().transactions() == 2u);
    CHECK(panel().commands_taken() == 2u);
}

TEST_CASE("adapter: MISO idles high through a write, and the buffer is a chip's") {
    fresh();

    // Nothing the panel drives: every byte of a write reads back as the
    // pull-up.
    link().select(true);
    link().dc(false);
    CHECK(link().byte(Dcs::ramwr) == 0xFF);
    link().dc(true);
    for (uint8_t i = 0; i < 12u; ++i) {
        CHECK(link().byte(static_cast<uint8_t>(0x40u + i)) == 0xFF);
    }
    link().select(false);

    // And with the select high nothing is driven either.
    CHECK(link().byte(0x00) == 0xFF);
}

TEST_CASE("adapter: the data past the buffer is dropped and counted") {
    using SmallLink = SimDcsSerial<Panel, 16>;
    fresh();
    set_window(0, 7, 0, 0);

    static SmallLink small{panel()};
    small.reset_counters();
    small.select(true);
    small.dc(false);
    (void)small.byte(Dcs::ramwr);
    small.dc(true);
    for (uint8_t i = 0; i < 24u; ++i) {
        (void)small.byte(0xFC);
    }
    small.select(false);

    // Sixteen bytes reached the core - five whole pixels and a byte -
    // and the eight past the buffer were counted and dropped.
    CHECK(small.overruns() == 8u);
    CHECK(panel().pixels_written() == 5u);
}

TEST_CASE("adapter: a read serves the bytes as they are clocked and the core sees the count") {
    fresh();
    wake();
    set_window(4, 11, 7, 7);
    uint8_t want[8u * 3u] = {};
    for (uint16_t i = 0; i < 8u; ++i) {
        put_pixel(want, i, pattern(static_cast<uint16_t>(4u + i), 7));
    }
    wire_write(Dcs::ramwr, want);

    // Three pixels clocked out of eight: the pointer moved by nine
    // bytes, and by nothing the dummy byte added.
    set_window(4, 11, 7, 7);
    uint8_t raw[10] = {};
    wire_read(Dcs::ramrd, raw);
    CHECK(raw[0] == Ili9481::memory_read_dummy_byte);
    uint32_t first = 0;
    CHECK(mismatches(want, raw + 1, 9, first) == 0u);
    CHECK(panel().read_pointer() == 9u);
    CHECK(panel().bytes_read() == 9u);

    // Three commands of the wake, two window registers twice, the write
    // and the read: nine, and not one more. THE PREFETCH THE ADAPTER
    // NEEDS in order to serve MISO before it knows how many bytes will
    // be clocked moves no counter at all.
    CHECK(panel().commands_taken() == 9u);
}
