// build: the front panel of a bench power supply, on the host.
//
// Two setpoints - volts and amps - each in a rounded frame, one of them
// selected. A button moves between them; the encoder's push walks the
// selected digit along, and turning it changes the value by that digit's
// weight. What it is for is to put the four pieces together for the
// first time: the drawing library, the framebuffer published to a
// viewer, the panel a viewer writes back, and the quadrature decoder.
//
// A VALUE IS A NUMBER AND NOT A ROW OF DIGITS. Turning the tens digit
// adds ten, so 09.999 becomes 10.000 the way an instrument does; editing
// characters would have given 00.999, which no instrument does. The
// display is then pure formatting.
//
// AND IT REDRAWS ONLY WHAT CHANGED, which is the whole economy of a
// write-only surface: a cell is rewritten opaque, so nothing needs
// erasing first and nothing needs reading back. Changing a digit touches
// ONE cell - or as many as a carry moved. Moving the selection touches
// two: the one that loses the highlight and the one that gains it. Only
// changing which setpoint is selected redraws a whole frame, because its
// colour changed. The Counting surface reports what each cost, so the
// economy is measured rather than claimed.
//
//   ./supply_panel            run it, and watch with: brio view supply
//   ./supply_panel --dump     draw one known state, print it, exit

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <array>
#include <string>

#include "gfx/counting.hpp"
#include "gfx/draw.hpp"
#include "gfx/font_5x7.hpp"
#include "gfx/surface.hpp"
#include "gfx/text.hpp"
#include "host/gfx_reference.hpp"
#include "host/platform.hpp"
#include "host/sim_display.hpp"
#include "host/sim_input.hpp"
#include "host/sim_panel.hpp"
#include "kernel/tenuto.hpp"
#include "util/input_scanner.hpp"
#include "util/quadrature.hpp"

using namespace brio;

namespace {

// ---------------------------------------------------------------------
// The board file's half: what this panel IS.
// ---------------------------------------------------------------------

constexpr Extent screen_w = 160;
constexpr Extent screen_h = 72;
using Screen = SimDisplay<Indexed8, screen_w, screen_h>;

// Four colours, named by role and not by shade - the shades are in the
// palette the viewer reads.
constexpr uint8_t col_bg = 0;
constexpr uint8_t col_frame = 1;
constexpr uint8_t col_ink = 2;
constexpr uint8_t col_live = 3;

// The controls, in the order the panel's snapshot carries them.
using SelectKey = SimButton<0>; ///< moves between the two setpoints
using PushKey = SimButton<1>;   ///< the encoder's own switch
using Knob = SimEncoder<0>;

Screen* screen = nullptr;
SimPanel* world = nullptr;

/// Ticks since the program started, from a real clock: this platform is
/// PACED, where the one the tests use is stepped by hand. Both are
/// HostPlatform; only the source of time and the world differ.
uint32_t elapsed_ms() {
    static timespec start{};
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (start.tv_sec == 0 && start.tv_nsec == 0) {
        start = now;
    }
    const int64_t ms = static_cast<int64_t>(now.tv_sec - start.tv_sec) * 1000 +
                       (now.tv_nsec - start.tv_nsec) / 1000000;
    return static_cast<uint32_t>(ms);
}

/**
 * THE WORLD TURNS IN idle(), and nowhere else. The kernel yields here
 * when it has nothing to do, which is where a target sleeps; here it is
 * where time advances and where the viewer's snapshot becomes the state
 * of the simulated contacts. Nothing is pushed into the kernel - the
 * scanner and the decoder read these levels on their own tick, exactly
 * as they would read pads.
 */
struct SimPlatform : HostPlatform {
    static void idle() {
        ticks = elapsed_ms();
        if (world != nullptr) {
            SelectKey::set(world->pressed(0));
            PushKey::set(world->pressed(1));
            Knob::set_shaft(world->shaft(0));
        }
        timespec nap{0, 500000}; // half a millisecond
        nanosleep(&nap, nullptr);
    }
};

// ---------------------------------------------------------------------
// The setpoints.
// ---------------------------------------------------------------------

/// One editable digit: which character cell shows it, and what turning
/// it is worth in milli-units.
struct Digit {
    uint8_t cell;
    int32_t weight;
};

constexpr Digit volt_digits[] = {{0, 10000}, {1, 1000}, {3, 100}, {4, 10}, {5, 1}};
constexpr Digit amp_digits[] = {{0, 1000}, {2, 100}, {3, 10}, {4, 1}};

struct Field {
    Coord x, y;
    Extent w, h;
    const char* unit;
    int32_t value;   ///< milli-units
    int32_t max;
    uint8_t cells;   ///< characters in the number, the point included
    const Digit* digits;
    uint8_t digit_count;
    uint8_t sel;     ///< which digit the encoder is on
    char shown[8];   ///< what is on the glass right now
};

Field fields[2] = {
    {2, 2, 156, 32, "V", 12000, 30000, 6, volt_digits, 5, 0, {0}},
    {2, 38, 156, 32, "A", 1500, 5000, 5, amp_digits, 4, 0, {0}},
};
uint8_t live = 0; ///< which setpoint the controls act on

constexpr Coord value_x = 44;
constexpr Coord label_x = 14;

Coord text_y(const Field& f) {
    return static_cast<Coord>(f.y + (f.h - Font5x7::cell_h) / 2);
}

/// The value as the characters that show it. Pure formatting: the model
/// is the number.
void format(const Field& f, char* out) {
    const uint8_t whole_cells = static_cast<uint8_t>(f.cells - 4);
    int32_t whole = f.value / 1000;
    int32_t frac = f.value % 1000;
    for (int i = whole_cells - 1; i >= 0; --i) {
        out[i] = static_cast<char>('0' + whole % 10);
        whole /= 10;
    }
    out[whole_cells] = '.';
    for (int i = 2; i >= 0; --i) {
        out[whole_cells + 1 + i] = static_cast<char>('0' + frac % 10);
        frac /= 10;
    }
    out[f.cells] = '\0';
}

/// Is cell `c` the digit the encoder is sitting on, and is this the
/// setpoint the controls act on?
bool highlighted(const Field& f, uint8_t c, bool is_live) {
    return is_live && f.digits[f.sel].cell == c;
}

template <Surface S>
void draw_cell(S& s, Field& f, uint8_t c, bool is_live) {
    const char ch[2] = {f.shown[c], '\0'};
    const Coord x = static_cast<Coord>(value_x + c * Font5x7::cell_w);
    const bool hi = highlighted(f, c, is_live);
    // Reverse video needs no compositing and no reading back: a glyph is
    // drawn foreground AND background, so swapping the two is the whole
    // of it.
    text<Font5x7>(s, x, text_y(f), std::string_view(ch, 1),
                  hi ? col_bg : col_ink, hi ? col_ink : col_bg);
}

/// The whole setpoint: its frame, its unit and every cell. Drawn when
/// the frame's colour changed, and once at the start.
template <Surface S>
void draw_field(S& s, Field& f, bool is_live) {
    fill_rect(s, f.x, f.y, f.w, f.h, col_bg);
    round_rect(s, f.x, f.y, f.w, f.h, 8, is_live ? col_live : col_frame);
    text<Font5x7>(s, label_x, text_y(f), f.unit, col_ink, col_bg);
    format(f, f.shown);
    for (uint8_t c = 0; c < f.cells; ++c) {
        draw_cell(s, f, c, is_live);
    }
    text<Font5x7>(s, static_cast<Coord>(value_x + f.cells * Font5x7::cell_w),
                  text_y(f), f.unit, col_ink, col_bg);
}

/// After the value moved: rewrite ONLY the cells whose character is not
/// what it was. One, normally; more when a carry ran.
template <Surface S>
void draw_changed(S& s, Field& f, bool is_live) {
    char now[8];
    format(f, now);
    for (uint8_t c = 0; c < f.cells; ++c) {
        if (now[c] != f.shown[c]) {
            f.shown[c] = now[c];
            draw_cell(s, f, c, is_live);
        }
    }
}

// ---------------------------------------------------------------------
// The active object.
// ---------------------------------------------------------------------

using Canvas = Counting<Screen::Surface>;
Screen::Surface* raw = nullptr;
Canvas* glass = nullptr;

void report(const char* what) {
    const DrawTally t = glass->take();
    printf("  %-22s %2u windows (%u rects, %u runs), %u pixels\n", what,
           t.windows(), t.rects, t.runs, t.pixels);
    fflush(stdout);
}

struct Ui : Fsm<Ui, InputEdge, Turned> {
    using Base = Fsm<Ui, InputEdge, Turned>;
    static inline EventQueue<Event, 8, SimPlatform> queue;

    static void init() { Base::start(&running); }
    static void dispatch(const Event& e) { Base::dispatch(e); }

    static void paint_all() {
        clear(*glass, col_bg);
        draw_field(*glass, fields[0], live == 0);
        draw_field(*glass, fields[1], live == 1);
        screen->publish();
    }

    static Status running(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](InputEdge k) {
                if (!k.active) {
                    return handled(); // act on the press, not the release
                }
                if (k.index == 0) {
                    // The frame's colour changes on both, so both are
                    // redrawn whole - the one case that costs a field.
                    live = static_cast<uint8_t>(1 - live);
                    draw_field(*glass, fields[0], live == 0);
                    draw_field(*glass, fields[1], live == 1);
                    report("select the other");
                } else {
                    // Two cells: the one losing the highlight and the
                    // one gaining it. Past the right, back to the left.
                    Field& f = fields[live];
                    const uint8_t was = f.digits[f.sel].cell;
                    f.sel = static_cast<uint8_t>((f.sel + 1) % f.digit_count);
                    draw_cell(*glass, f, was, true);
                    draw_cell(*glass, f, f.digits[f.sel].cell, true);
                    report("move the digit");
                }
                screen->publish();
                return handled();
            },
            [](Turned t) {
                Field& f = fields[live];
                const int32_t step = f.digits[f.sel].weight * t.detents;
                int32_t v = f.value + step;
                v = v < 0 ? 0 : (v > f.max ? f.max : v);
                if (v == f.value) {
                    return handled(); // at an end stop: nothing moved
                }
                f.value = v;
                draw_changed(*glass, f, true);
                report("turn one detent");
                screen->publish();
                return handled();
            });
    }
};

using Scanner = InputScanner<SimPlatform, Subscribers<Ui>, ScanConfig{}, SelectKey,
                             PushKey>;
using Decoder = Quadrature<SimPlatform, Subscribers<Ui>, Knob::A, Knob::B>;
using Loop = Tenuto<SimPlatform, Scanner, Decoder, Ui>;

} // namespace

int main(int argc, char** argv) {
    const bool dump = argc > 1 && std::string(argv[1]) == "--dump";

    Screen display("supply");
    SimPanel panel("supply");
    screen = &display;
    world = &panel;

    // A panel says what its controls ARE; a viewer only says what they
    // are doing.
    panel.name_button(0, "V / A");
    panel.name_button(1, "Digit");
    panel.name_shaft(0, "Adjust", 1, Decoder::counts_per_detent);

    display.set_palette(col_bg, 0x10, 0x14, 0x1C);
    display.set_palette(col_frame, 0x50, 0x58, 0x60);
    display.set_palette(col_ink, 0xE8, 0xF0, 0xF8);
    display.set_palette(col_live, 0xFF, 0xB0, 0x30);

    Screen::Surface surface = display.surface();
    Canvas counted(surface);
    raw = &surface;
    glass = &counted;

    Loop::init_all();
    // The AO contract's init() takes no arguments, so the application
    // arms the two periodic readers right after - here, every tick.
    Scanner::start_every(1);
    Decoder::start_every(1);
    Ui::paint_all();

    if (dump) {
        // One known state, drawn and printed: what the screen holds,
        // with no window and no viewer.
        printf("%s", ascii(surface).c_str());
        report("first paint");

        // Then what each interaction costs, which is the point: a full
        // repaint is the expensive thing, and almost nothing is one.
        printf("\nwhat an interaction costs:\n");
        Ui::dispatch(Ui::Event{Turned{1}});          // 12.000 -> 22.000
        Ui::dispatch(Ui::Event{Turned{-1}});         // and back
        fields[0].sel = 1;                           // onto the units digit
        Ui::dispatch(Ui::Event{Turned{-3}});         // 12.000 -> 9.000: a carry
        Ui::dispatch(Ui::Event{InputEdge{1, true}}); // move the digit
        Ui::dispatch(Ui::Event{InputEdge{0, true}}); // the other setpoint
        return 0;
    }

    printf("supply_panel: watch it with   brio view supply\n");
    printf("  %s = %s, %s = %s, %s = %s\n", "button 0", panel.button_name(0).c_str(),
           "button 1", panel.button_name(1).c_str(), "shaft 0",
           panel.shaft_name(0).c_str());
    fflush(stdout);

    for (;;) {
        TimeEvents<SimPlatform>::process();
        if (!Loop::step()) {
            Loop::idle_if_empty();
        }
    }
}
