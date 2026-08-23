// spi_paint - draw on the ILI9481 with the XPT2046 pen: the
// responsiveness testbed for the touch->pixel chain.
//
//   Touch   - polls the pen at ~100 Hz (poll_ms below). Pressed
//             samples are mapped to screen coordinates and posted to
//             the painter; a release posts PenUp.
//   Painter - clears the panel white, then ink: each sample extends
//             the current stroke with overlapping 4x4 dots (Chebyshev
//             steps), coalescing samples that arrive while a segment
//             is still being drawn (latest target wins - interpolation
//             fills the gap).
//
// The measurement side (the actual point of the app): every stroke
// prints its sample count, duration, effective sample rate and how
// many samples were coalesced. That answers "is 20 Hz polling enough
// or do we need the PEN interrupt?" with numbers: PEN would not raise
// the sample rate - polling faster does (2% of bus at 100 Hz); PEN
// only buys idle power and first-dot latency below one poll period.
//
// Touch calibration: raw_min/raw_max below are typical XPT2046 panel
// bounds; the first sample of each stroke also prints its RAW values.
// Tap the four corners and adjust raw bounds / swap_xy / flip flags if
// the ink lands elsewhere - two flashes and it converges.
//
// Wiring per README bench map (display + touch on the shared bus).

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrdx/clock.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/spi.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/wire.hpp"

using P = brio::AvrPlatform;

// The clock: the ONE truth about CLK_PER for every driver of this
// target (avrdx/clock.hpp). 24 MHz crystal on PA0/PA1, OSCHF fallback at
// the same rate; `clock` is an empty tag passed to driver inits.
using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

using SpiHw = brio::Spi<0>;
using Bus = brio::SpiBus<SpiHw, P>;

using CsPin = brio::Pin<'D', 0>;
using DcPin = brio::Pin<'D', 1>;
using RstPin = brio::Pin<'D', 2>;
using TcsPin = brio::Pin<'D', 5>;

// ---- geometry and pen tuning ------------------------------------------------
constexpr uint16_t width = 320;                // columns (CASET)
constexpr uint16_t height = 480;               // pages (PASET)
constexpr uint16_t row_bytes = width * 3;

constexpr uint16_t poll_ms = 10;               // ~100 Hz pen sampling
constexpr uint16_t z_threshold = 250;  // real presses read 800+, ghosts ~130

constexpr uint8_t dot_px = 4;                  // ink dot side
constexpr uint8_t dot_step = 3;                // < dot_px: overlapping stroke

// Raw-to-screen calibration, per axis (from bench captures: stroke
// starts spanned x 688..3778, y 290..3594 - edges lie a bit beyond)
constexpr uint16_t raw_x_min = 600;
constexpr uint16_t raw_x_max = 3850;
constexpr uint16_t raw_y_min = 250;
constexpr uint16_t raw_y_max = 3750;
// Corner-tap calibration (beacon at panel origin seen top-left; tap
// there read raw x~3200 y~290): view aligned with panel, touch X
// mirrored, touch Y straight.
constexpr bool swap_xy = false;
constexpr bool flip_x = true;
constexpr bool flip_y = false;

// ---- events -----------------------------------------------------------------
struct Poll {};
struct Tick {};
struct Sample { uint16_t x, y; };              // screen coords, pen pressed
struct PenUp {};

// ============================ painter client ================================

struct Painter : brio::Fsm<Painter, Tick, Sample, PenUp, brio::SpiDone> {
    static inline brio::EventQueue<Event, 6, P> queue;

    struct Pt { int16_t x, y; };

    static inline uint16_t row = 0;
    static inline uint8_t row_buf[row_bytes];  // white rows for the clear
    static inline uint8_t dot_buf[dot_px * dot_px * 3];
    static inline uint8_t params[4];
    static inline uint8_t cur_cmd = 0;

    static inline Pt cur{0, 0};                // last inked position
    static inline Pt tgt{0, 0};                // latest pen position
    static inline bool pen_down = false;
    static inline bool drawing = false;        // a dot chain is in flight
    static inline uint8_t dot_phase = 0;       // 0 CASET, 1 PASET, 2 RAMWR

    // per-stroke stats
    static inline uint16_t samples = 0;
    static inline uint16_t coalesced = 0;
    static inline uint32_t t_down = 0;

    static void init() {
        CsPin::set();  CsPin::output();
        DcPin::set();  DcPin::output();
        RstPin::set(); RstPin::output();
        brio::Pin<'D', 3>::set();              // deselect the MCP3550
        brio::Pin<'D', 3>::output();
        for (uint16_t i = 0; i < row_bytes; ++i) {
            row_buf[i] = 0xFC;                 // white
        }
        for (uint16_t i = 0; i < sizeof(dot_buf); ++i) {
            dot_buf[i] = 0x00;                 // black ink
        }
        start(&waking);
    }

    // reset -> SLPOUT -> (150 ms) -> DISPON -> (25 ms) -> INVON
    // (the 9481 panel polarity quirk: without INVON the white canvas
    // would render black), then clear -> painting.
    static inline brio::TimeEvent<P, Painter, Tick> timer{Tick{}};
    static inline uint8_t phase = 0;

    static Status waking(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                phase = 0;
                RstPin::clear();
                timer.arm(brio::ticks_from_ms<P>(5));
                return handled();
            },
            [](Tick) {
                switch (phase) {
                    case 0:
                        RstPin::set();
                        timer.arm(brio::ticks_from_ms<P>(150));
                        phase = 1;
                        break;
                    case 1: send(0x11, nullptr, 0); phase = 2; break;  // SLPOUT
                    case 3: send(0x29, nullptr, 0); phase = 4; break;  // DISPON
                    case 5: send(0x21, nullptr, 0); phase = 6; break;  // INVON
                    default: break;
                }
                return handled();
            },
            [](brio::SpiDone) {
                switch (phase) {
                    case 2:                    // SLPOUT sent: settle
                        timer.arm(brio::ticks_from_ms<P>(150));
                        phase = 3;
                        break;
                    case 4:                    // DISPON sent: settle
                        timer.arm(brio::ticks_from_ms<P>(25));
                        phase = 5;
                        break;
                    default:                   // INVON sent: go clear
                        return transition(&clearing);
                }
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }

    static Status clearing(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                phase = 0;
                set_window(0x2A, 0, width - 1);
                return handled();
            },
            [](brio::SpiDone) {
                switch (phase) {
                    case 0:
                        ++phase;
                        set_window(0x2B, 0, height - 1);
                        break;
                    case 1:
                        ++phase;
                        row = 0;
                        send(0x2C, row_buf, row_bytes);
                        break;
                    default:
                        if (++row < height) {
                            send(0x3C, row_buf, row_bytes);
                        } else {
                            return transition(&marking);
                        }
                        break;
                }
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }

    // Orientation beacon: a black 60x20 rectangle at PANEL (0,0), long
    // side along the CASET axis. Where the user sees it (which corner,
    // horizontal or vertical) pins the panel coordinate system to the
    // bench view - calibration then follows from four corner taps.
    static constexpr uint16_t mark_w = 60, mark_h = 20;

    static Status marking(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                phase = 0;
                for (uint16_t i = 0; i < mark_w * 3; ++i) {
                    row_buf[i] = 0x00;         // black beacon rows
                }
                set_window(0x2A, 0, mark_w - 1);
                return handled();
            },
            [](brio::SpiDone) {
                switch (phase) {
                    case 0:
                        ++phase;
                        set_window(0x2B, 0, mark_h - 1);
                        break;
                    case 1:
                        ++phase;
                        row = 0;
                        send(0x2C, row_buf, mark_w * 3);
                        break;
                    default:
                        if (++row < mark_h) {
                            send(0x3C, row_buf, mark_w * 3);
                        } else {
                            return transition(&painting);
                        }
                        break;
                }
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }

    static Status painting(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                brio::print(serial, "canvas ready - draw!", brio::crlf);
                pen_down = false;
                drawing = false;
                return handled();
            },
            [](Sample s) {
                const Pt p{static_cast<int16_t>(s.x),
                           static_cast<int16_t>(s.y)};
                if (!pen_down) {               // stroke start
                    pen_down = true;
                    samples = 1;
                    coalesced = 0;
                    t_down = P::now();
                    cur = tgt = p;
                    begin_dot(cur);
                    return handled();
                }
                ++samples;
                tgt = p;                       // latest target wins
                if (drawing) {
                    ++coalesced;               // will be interpolated over
                } else if (!advance()) {
                    // same pixel: nothing to ink
                }
                return handled();
            },
            [](PenUp) {
                pen_down = false;
                const uint32_t dt = P::now() - t_down;
                const uint32_t ms = dt * 1000u / P::ticks_per_second;
                brio::print(serial, "stroke: ", samples, " samples in ",
                            ms, " ms (~",
                            ms ? samples * 1000u / ms : 0,
                            " Hz), coalesced ", coalesced, brio::crlf);
                return handled();
            },
            [](brio::SpiDone) {
                switch (dot_phase) {
                    case 0:
                        dot_phase = 1;
                        set_window(0x2B, static_cast<uint16_t>(tgt_dot.y),
                                   static_cast<uint16_t>(tgt_dot.y) + dot_px - 1);
                        break;
                    case 1:
                        dot_phase = 2;
                        send(0x2C, dot_buf, sizeof(dot_buf));
                        break;
                    default:                   // dot inked
                        drawing = false;
                        advance();
                        break;
                }
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }

private:
    static inline Pt tgt_dot{0, 0};            // dot being inked right now

    /// Move cur one Chebyshev step toward tgt and ink there.
    /// Returns false when there is nothing left to draw.
    static bool advance() {
        if (cur.x == tgt.x && cur.y == tgt.y) {
            return false;
        }
        cur.x += step_toward(cur.x, tgt.x);
        cur.y += step_toward(cur.y, tgt.y);
        begin_dot(cur);
        return true;
    }

    static int16_t step_toward(int16_t from, int16_t to) {
        const int16_t d = static_cast<int16_t>(to - from);
        if (d > dot_step) return dot_step;
        if (d < -dot_step) return static_cast<int16_t>(-dot_step);
        return d;
    }

    static void begin_dot(Pt p) {
        // clamp so the dot window stays on the panel
        if (p.x > static_cast<int16_t>(width - dot_px)) p.x = width - dot_px;
        if (p.y > static_cast<int16_t>(height - dot_px)) p.y = height - dot_px;
        if (p.x < 0) p.x = 0;
        if (p.y < 0) p.y = 0;
        tgt_dot = p;
        drawing = true;
        dot_phase = 0;
        set_window(0x2A, static_cast<uint16_t>(p.x),
                   static_cast<uint16_t>(p.x) + dot_px - 1);
    }

    static void send(uint8_t c, const uint8_t* data, uint16_t n) {
        cur_cmd = c;
        brio::post<Bus>(SpiHw::Request{
            CsPin::ref(), DcPin::ref(),
            &cur_cmd, 1,
            data, nullptr, n,
            brio::reply_to<Painter, brio::SpiDone>(),
            brio::SpiClock::div4,
            SPI_MODE_0_gc, true});             // polled bulk
    }

    static void set_window(uint8_t c, uint16_t first, uint16_t last) {
        brio::store_be16(params, first);
        brio::store_be16(params + 2, last);
        send(c, params, 4);
    }
};

// ============================= touch client =================================

struct Touch : brio::Fsm<Touch, Poll, brio::SpiDone> {
    static inline brio::EventQueue<Event, 3, P> queue;
    static inline brio::TimeEvent<P, Touch, Poll> cadence{Poll{}};

    // One conversion is 3 bytes [ctrl, 0, 0]; a BURST is `burst_n`
    // conversions in ONE CS window (~130 us at 1.5 MHz): the mux and
    // plate drivers stay settled between conversions, and a trimmed
    // mean over the tail kills the single-shot noise that makes raw
    // XPT2046 strokes zigzag.
    static constexpr uint8_t burst_n = 8;
    static constexpr uint8_t burst_skip = 2;   // mux settling: discard
    static constexpr uint8_t burst_bytes = burst_n * 3;

    static inline uint8_t phase = 0;
    static inline uint8_t tx[burst_bytes];
    static inline uint8_t rx[burst_bytes];
    static inline uint16_t z = 0;
    static inline uint16_t xr = 0;
    static inline uint16_t yr = 0;
    static inline bool was_pressed = false;

    static void init() {
        TcsPin::set();
        TcsPin::output();
        start(&polling);
    }

    static Status polling(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                cadence.arm_every(brio::ticks_from_ms<P>(poll_ms));
                return handled();
            },
            [](Poll) {
                phase = 0;
                convert(0xB1);                 // Z1, keep powered
                return handled();
            },
            [](brio::SpiDone d) {
                if (d.status != brio::spi_ok) {
                    return handled();          // lost this slot, fine
                }
                switch (phase) {
                    case 0:
                        z = value_at(0);
                        if (z > z_threshold) {
                            phase = 1;
                            burst(0xD1, 0xD1); // X burst, keep powered
                        } else {
                            if (was_pressed) {
                                was_pressed = false;
                                brio::post<Painter>(PenUp{});
                            }
                        }
                        break;
                    case 1:
                        xr = trimmed_mean();
                        phase = 2;
                        burst(0x91, 0x90);     // Y burst, power down last
                        break;
                    default:
                        yr = trimmed_mean();
                        on_sample();
                        break;
                }
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }

private:
    static void on_sample() {
        if (!was_pressed) {
            was_pressed = true;
            brio::print(serial, "pen raw x=", xr, " y=", yr, " z=", z,
                        brio::crlf);
        }
        const uint16_t a = map_axis(swap_xy ? yr : xr, raw_x_min, raw_x_max,
                                    width - dot_px, flip_x);
        const uint16_t b = map_axis(swap_xy ? xr : yr, raw_y_min, raw_y_max,
                                    height - dot_px, flip_y);
        brio::post<Painter>(Sample{a, b});
    }

    static uint16_t map_axis(uint16_t raw, uint16_t lo, uint16_t hi,
                             uint16_t out_max, bool flip) {
        if (raw < lo) raw = lo;
        if (raw > hi) raw = hi;
        uint16_t v = static_cast<uint16_t>(
            static_cast<uint32_t>(raw - lo) * out_max / (hi - lo));
        return flip ? static_cast<uint16_t>(out_max - v) : v;
    }

    static uint16_t value_at(uint8_t i) {
        return static_cast<uint16_t>(
            (brio::load_be16(rx + 3 * i + 1) >> 3) & 0x0FFF);
    }

    /// Robust burst estimate: skip the settling head, then drop the
    /// min and max of the tail and average the rest.
    static uint16_t trimmed_mean() {
        uint16_t lo = 0xFFFF, hi = 0;
        uint32_t sum = 0;
        for (uint8_t i = burst_skip; i < burst_n; ++i) {
            const uint16_t v = value_at(i);
            sum += v;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        return static_cast<uint16_t>((sum - lo - hi) /
                                     (burst_n - burst_skip - 2));
    }

    /// Single conversion (pressure gate): 3-byte transfer.
    static void convert(uint8_t ctrl) {
        tx[0] = ctrl;
        tx[1] = 0;
        tx[2] = 0;
        post_xfer(3);
    }

    /// burst_n conversions in one CS window; the last control byte may
    /// differ (0x90 = same channel + power down on the way out).
    static void burst(uint8_t ctrl, uint8_t last_ctrl) {
        for (uint8_t i = 0; i < burst_n; ++i) {
            tx[3 * i] = (i == burst_n - 1) ? last_ctrl : ctrl;
            tx[3 * i + 1] = 0;
            tx[3 * i + 2] = 0;
        }
        post_xfer(burst_bytes);
    }

    static void post_xfer(uint16_t n) {
        brio::post<Bus>(SpiHw::Request{
            TcsPin::ref(), {},
            nullptr, 0,
            tx, rx, n,
            brio::reply_to<Touch, brio::SpiDone>(),
            brio::SpiClock::div16});           // 1.5 MHz, ISR path
    }
};

} // namespace

// ---- target glue ------------------------------------------------------------
ISR(SPI0_INT_vect) {
    if (SpiHw::isr()) {
        brio::post<Bus>(brio::TransferDone{brio::spi_ok});
    }
}
ISR(USART2_RXC_vect) { Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    SysClock::init();
    Serial::init(clock, 460800);
    SpiHw::init(clock);
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf,
                "spi_paint: draw on the ILI9481 with the XPT2046 pen",
                brio::crlf);

    brio::Kernel<P, Bus, Touch, Painter>::run();
}
