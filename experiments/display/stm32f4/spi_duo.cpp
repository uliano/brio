// spi_duo - TWO DEVICES ON ONE SPI THROUGH BRIO'S ARBITER, on the black
// pill: the ILI9481 panel and the XPT2046 touch controller of the same
// module share SCK/MOSI/MISO, each with its own chip select, and two
// active objects post their transactions to ONE SpiBus (util/spi_bus.hpp
// = BusMaster). The bus serves them in order, reprogramming the
// peripheral per request - 6 MHz mode 0 for a row of pixels, 1.5 MHz
// mode 0 for a touch conversion - so nothing above the arbiter knows
// the other device exists.
//
//   Painter  owns the glass and THE GAME (the F469 suite's finger-chases-
//            a-square, brought to a resistive panel): a white square at
//            ten random places, every tap's raw coordinates recorded
//            against the square's centre, from the third tap the best of
//            the two axis assignments with a least-squares line per axis
//            says where the tap was understood (a red cross), after the
//            tenth the fit is printed and the pen paints dots with it.
//            Every mark is a rectangle fill queued in a small FIFO, one
//            row a request through the DMA engines at 6 MHz; when the
//            FIFO is empty a band of changing colour sweeps the bottom
//            strip, so the bus is never idle
//   Touch    polls the XPT2046 every 20 ms at 1.5 MHz: a Z1 pressure
//            gate with the chip powered down between polls (which is
//            what keeps PENIRQ alive), then X and Y as bursts of eight
//            conversions in one select window, trimmed means; a stroke
//            that begins after three idle polls is a Tap, every sample a
//            Sample
//   Bus      SpiBus<Host, P, 8>: the FIFO of pending requests, one in
//            flight, TransferDone off the SPI's and the streams' vectors
//
// THE MEASUREMENT: the touch's latency is bounded by one row of the
// painter (about 2 ms at 6 MHz) because the arbiter is a FIFO and the
// painter posts one request at a time; every two seconds the console
// prints the rows painted, the polls and samples taken and the
// requests the arbiter REJECTED (a full FIFO) - which should stay zero.
// And the game's RESULT lines are the calibration: the axis assignment,
// the two lines, the error, and the raw values at the logical edges.
//
// The rotation is the probe's r90 map (landscape, 480 x 320) with this
// module's two facts folded in: column 319 at the left of the glass,
// BGR in the bytes.
//
// Wiring (3.3 V): SPI1 SCK PA5 / MISO PA6 (SDO and T_DO) / MOSI PA7
// (SDI and T_DIN); display CS PB2, RESET PB1, DC PB0; touch T_CS PA4,
// T_IRQ PA1; VCC and LED on 3.3 V. Console on the USB-C (CDC ACM).
//
// build: boards = f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usb.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"
#include "util/wire.hpp"

namespace {

using namespace brio;

using P = Stm32f4Platform<>;
using SysClock = Clock<ClockSource::pll_hse, 96'000'000, 25'000'000>;
constexpr SysClock clock;
static_assert(SysClock::usb_hz == otg_clock_hz, "this rate must give the controller 48 MHz");

// ---- the console: the chip's own USB, output only ------------------------------
using Usb = UsbFs;
using Serial = UsbCdcAcm<Usb, P, 0, 1, 2, 256, 1024>;
constexpr Serial serial;

struct Descriptors {
    static constexpr auto device =
        usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Serial::descriptor_bytes, Serial::interface_count),
                   Serial::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto manufacturer = usb_string_descriptor("brio");
    static constexpr auto product = usb_string_descriptor("brio console");
    static constexpr auto serial_number = usb_string_descriptor("stm32f4");
    static std::span<const uint8_t> string(uint8_t index) {
        switch (index) {
        case 0: return language;
        case 1: return manufacturer;
        case 2: return product;
        case 3: return serial_number;
        default: return {};
        }
    }
};
using Device = UsbDevice<Usb, Descriptors, Serial>;

/// A line to the console only while a terminal holds DTR: print()
/// blocks until the transport takes every byte, and a port nobody
/// reads would stop the kernel.
template <typename... Args>
void say(const Args&... args) {
    if (Serial::configured() && Serial::dtr()) {
        print(serial, args..., crlf);
    }
}

// ---- the wires ------------------------------------------------------------------
using Led = Pin<'C', 13>;   // lit when low
using Cs = Pin<'B', 2>;
using Rst = Pin<'B', 1>;
using Dc = Pin<'B', 0>;
using Tcs = Pin<'A', 4>;
using Tirq = Pin<'A', 1>;

constexpr SpiPins spi_pins{
    .sck = {'A', 5, PinFunction::af5},
    .miso = {'A', 6, PinFunction::af5},
    .mosi = {'A', 7, PinFunction::af5},
};
using TxEngine = DmaTxEngine<2, 3, 3>;   // SPI1_TX: RM0383 table 28
using RxEngine = DmaRxEngine<2, 0, 3>;   // SPI1_RX
/// The two DMA engines carry every data phase; the interrupt-style
/// engined path is byte-exact against this panel at 12 MHz with the
/// SCK pad at medium (the probe's letter v).
using Host = SpiHost<1, spi_pins, TxEngine, RxEngine>;
/// The panel's bring-up runs on the plain pump host - the path the probe
/// proved - before the engined one takes the instance over.
using BootHost = SpiHost<1, spi_pins>;
using Bus = SpiBus<Host, P, 8>;
/// The vectors post TransferDone to the bus AO only once the kernel
/// runs: the panel's bring-up drives the host directly before that, and
/// a completion posted into a queue nobody has started would be served
/// as a ghost when the arbiter comes up.
bool bus_live = false;
bool host_ok = false;

/// 6 MHz: byte-exact through the engines in this program (its own
/// report reads the square back). 12 MHz needs a pad at medium for an
/// interrupt-style request (the probe's letters j and m, where it is
/// exact) and still lands nothing HERE - measured with the touch's
/// requests switched off, and with the probe's own passing pads (SCK
/// very_high, MOSI medium): the square reads back as the dummy byte
/// and zeros. So the actor is this program's request path under the
/// kernel and neither the pads nor the interleaving - open; both pads
/// are set to medium all the same, which costs nothing at 6 MHz.
constexpr SpiClock panel_rate = SpiClock::div16;
constexpr SpiClock touch_rate = SpiClock::div64;  // 1.5 MHz
constexpr bool touch_enabled = true;

// ---- the panel, the probe's r90 map -----------------------------------------------
namespace ili {

constexpr uint16_t width = 320;    // physical columns
constexpr uint16_t height = 480;   // physical pages
constexpr uint16_t lw = height;    // logical, landscape
constexpr uint16_t lh = width;

constexpr uint8_t slpout = 0x11;
constexpr uint8_t invon = 0x21;
constexpr uint8_t dispon = 0x29;
constexpr uint8_t caset = 0x2A;
constexpr uint8_t paset = 0x2B;
constexpr uint8_t ramwr = 0x2C;
constexpr uint8_t madctl = 0x36;
constexpr uint8_t ramwr_continue = 0x3C;

/// r90 clockwise over a glass that shows column 319 at its left: logical
/// x runs down the pages, logical y runs along the columns from 0 up;
/// B5 makes the counter step the pages first.
constexpr uint8_t madctl_r90 = 0x20;

void put_pixel(uint8_t* row, uint16_t i, uint32_t rgb) {   // BGR on this module
    row[i * 3u] = static_cast<uint8_t>(rgb);
    row[i * 3u + 1u] = static_cast<uint8_t>(rgb >> 8);
    row[i * 3u + 2u] = static_cast<uint8_t>(rgb >> 16);
}

/// The CASET and PASET parameters for a logical rectangle under r90:
/// CASET takes the pages (logical x), PASET the columns (logical y).
void window_params(uint16_t lx, uint16_t ly, uint16_t w, uint16_t h, uint8_t* cas, uint8_t* pas) {
    const uint16_t p0 = lx, p1 = static_cast<uint16_t>(lx + w - 1u);
    const uint16_t c0 = ly, c1 = static_cast<uint16_t>(ly + h - 1u);
    cas[0] = static_cast<uint8_t>(p0 >> 8); cas[1] = static_cast<uint8_t>(p0);
    cas[2] = static_cast<uint8_t>(p1 >> 8); cas[3] = static_cast<uint8_t>(p1);
    pas[0] = static_cast<uint8_t>(c0 >> 8); pas[1] = static_cast<uint8_t>(c0);
    pas[2] = static_cast<uint8_t>(c1 >> 8); pas[3] = static_cast<uint8_t>(c1);
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// Before the kernel runs: a polled command straight into the host.
uint8_t bring_up_status[8];
uint8_t bring_up_count = 0;
void command_now(uint8_t cmd, const uint8_t* params, uint16_t n) {
    static uint8_t c = 0;
    c = cmd;
    BootHost::Request r{};
    r.cs = Cs::ref();
    r.dc = Dc::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&c));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(params);
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.len = n;
    r.clock = panel_rate;
    r.mode = SpiMode::mode0;
    r.polled = true;
    const bool sync = BootHost::start(r);
    if (bring_up_count < 8u) {
        bring_up_status[bring_up_count++] = static_cast<uint8_t>((sync ? 0x00u : 0x80u) | BootHost::status());
    }
}

void bring_up() {
    Rst::clear();
    wait_ms(10);
    Rst::set();
    wait_ms(150);
    command_now(slpout, nullptr, 0);
    wait_ms(150);
    command_now(dispon, nullptr, 0);
    wait_ms(25);
    command_now(invon, nullptr, 0);
    const uint8_t m = madctl_r90;
    command_now(madctl, &m, 1);
}

}  // namespace ili

// ---- events -----------------------------------------------------------------
struct Poll {};
struct Report {};
struct Sample {          // the pen is down: raw controller coordinates
    uint16_t rx;
    uint16_t ry;
};
struct Tap {             // the first sample of a stroke
    uint16_t rx;
    uint16_t ry;
    uint16_t z;
};
struct PenUp {};

// ---- the painter: a FIFO of rectangles, and the chase --------------------------
/// The Painter owns the panel and the GAME: a white square at ten random
/// places, each tap's raw coordinates recorded against the square's
/// centre; from the third tap the best of the two axis assignments
/// (swapped or not) with a least-squares line per axis says where the
/// tap was understood, marked by a red cross; after the tenth the fit
/// is printed and the pen paints dots with it. Everything on the glass
/// is a RECTANGLE FILL through the bus, one row a request, queued in a
/// small FIFO; when the FIFO is empty the sweep band in the bottom strip
/// keeps the bus busy.
struct Restart {};
struct ShowResults {};

struct Painter : Fsm<Painter, Tap, Sample, PenUp, SpiDone, Restart, ShowResults> {
    static inline EventQueue<Event, 8, P> queue;

    static constexpr uint16_t band_rows = 4;
    static constexpr uint16_t strip_y = ili::lh - 24;   // the sweep's strip: y 296..319
    static constexpr uint16_t side = 60;
    static constexpr uint8_t targets = 10;
    static constexpr uint16_t dot = 9;
    static constexpr uint16_t row_bytes = ili::lw * 3u;

    struct Fill {
        uint16_t x, y, w, h;
        uint32_t colour;
        bool verify;   // a READ of the row instead: how many pixels are `colour`
    };
    static inline uint8_t read_buf[ili::lw * 3u + 8u];
    static inline uint32_t verified_ok = 0, verified_bad = 0;
    static inline uint8_t verify_head[8];
    struct VerifyRecord {
        uint16_t x, y, good, w;
        uint8_t head[4];
    };
    static inline VerifyRecord verifies[8];
    static inline uint8_t verify_count = 0;
    static constexpr uint8_t fifo_depth = 12;
    static inline Fill fifo[fifo_depth];
    static inline uint8_t fifo_head = 0, fifo_count = 0;
    static inline Fill cur{};
    static inline bool busy = false;
    static inline uint8_t step = 0;
    static inline uint16_t row = 0;

    static inline uint16_t band_y = strip_y;
    static inline uint8_t hue = 0;
    static inline uint32_t rows_painted = 0, bands_painted = 0, dots_painted = 0, fills_dropped = 0, failed = 0;

    static inline uint8_t cmd = 0;
    static inline uint8_t cas[4];
    static inline uint8_t pas[4];
    static inline uint8_t row_buf[row_bytes];

    // The game.
    struct TapRecord {
        uint16_t rx, ry;   // the controller's
        uint16_t sx, sy;   // the square's centre, logical
        uint16_t z;        // Z1 at the first sample
    };
    static inline TapRecord taps[targets];
    static inline uint8_t n_taps = 0;
    static inline uint16_t sx = 0, sy = 0;      // the square's centre
    static inline bool cross_shown = false;
    static inline int32_t cx = 0, cy = 0;       // the cross's centre
    static inline bool calibrated = false;
    static inline uint32_t seed = 0x2545F491u;
    static inline bool dot_pending = false;
    static inline Sample dot_target{0, 0};

    /// x = ax * u + bx, y = ay * v + by with (u, v) = (rx, ry) or (ry, rx).
    struct Fit {
        bool swap;
        float ax, bx, ay, by;
        uint32_t rms, worst;
    };
    static inline Fit fit{false, 0.f, 0.f, 0.f, 0.f, 0, 0};

    static void init() { start(&running); }

    static Status running(const Event& e) {
        return match(e,
            [](Entry) {
                restart();
                enqueue({static_cast<uint16_t>(sx - side / 2u), sy, side, 1, 0xFCFCFC, true});
                pump();
                return handled();
            },
            [](Tap t) {
                on_tap(t);
                return handled();
            },
            [](Sample s) {
                if (calibrated) {
                    dot_target = s;
                    dot_pending = true;
                    pump();
                }
                return handled();
            },
            [](PenUp) { return handled(); },
            [](SpiDone d) {
                advance(d.status == spi_ok);
                return handled();
            },
            [](Restart) {
                restart();
                return handled();
            },
            [](ShowResults) {
                print_results();
                return handled();
            },
            [](auto) { return unhandled(); });
    }

    // ---- the fit ---------------------------------------------------------------------
    static int32_t mapped_x(const Fit& f, uint16_t rx, uint16_t ry) {
        const float u = static_cast<float>(f.swap ? ry : rx);
        return static_cast<int32_t>(f.ax * u + f.bx);
    }
    static int32_t mapped_y(const Fit& f, uint16_t rx, uint16_t ry) {
        const float v = static_cast<float>(f.swap ? rx : ry);
        return static_cast<int32_t>(f.ay * v + f.by);
    }

    /// A least-squares line through (u_i, t_i): slope and intercept.
    static void line_fit(const uint16_t* u, const uint16_t* t, uint8_t n, float& a, float& b) {
        float su = 0.f, st = 0.f, suu = 0.f, sut = 0.f;
        for (uint8_t i = 0; i < n; ++i) {
            su += u[i];
            st += t[i];
            suu += static_cast<float>(u[i]) * u[i];
            sut += static_cast<float>(u[i]) * t[i];
        }
        const float den = n * suu - su * su;
        if (den > -1.f && den < 1.f) {
            a = 0.f;
            b = st / n;
            return;
        }
        a = (n * sut - su * st) / den;
        b = (st - a * su) / n;
    }

    /// The best of the two axis assignments over the taps `use` marks.
    static Fit best_fit(const bool* use, uint8_t n) {
        Fit best{false, 0.f, 0.f, 0.f, 0.f, 0, 0};
        uint32_t best_err = 0xFFFFFFFFu;
        for (uint8_t c = 0; c < 2u; ++c) {
            Fit f{c != 0u, 0.f, 0.f, 0.f, 0.f, 0, 0};
            uint16_t u[targets], v[targets], tx[targets], ty[targets];
            uint8_t m = 0;
            for (uint8_t i = 0; i < n; ++i) {
                if (!use[i]) {
                    continue;
                }
                u[m] = f.swap ? taps[i].ry : taps[i].rx;
                v[m] = f.swap ? taps[i].rx : taps[i].ry;
                tx[m] = taps[i].sx;
                ty[m] = taps[i].sy;
                ++m;
            }
            if (m < 2u) {
                return best;
            }
            line_fit(u, tx, m, f.ax, f.bx);
            line_fit(v, ty, m, f.ay, f.by);
            uint32_t err = 0;
            for (uint8_t i = 0; i < n; ++i) {
                if (!use[i]) {
                    continue;
                }
                const uint32_t d2 = residual2(f, i);
                err += d2;
                const uint32_t d = isqrt32(d2);
                f.worst = d > f.worst ? d : f.worst;
            }
            f.rms = isqrt32(err / m);
            if (err < best_err) {
                best = f;
                best_err = err;
            }
        }
        return best;
    }

    static uint32_t residual2(const Fit& f, uint8_t i) {
        const int32_t dx = mapped_x(f, taps[i].rx, taps[i].ry) - taps[i].sx;
        const int32_t dy = mapped_y(f, taps[i].rx, taps[i].ry) - taps[i].sy;
        return static_cast<uint32_t>(dx * dx + dy * dy);
    }

    /// The fit over every tap, then - with six or more - again without
    /// the two that sit farthest from it: a finger that landed early
    /// is thrown out by its own distance.
    static Fit best_fit(uint8_t n) {
        bool use[targets];
        for (uint8_t i = 0; i < targets; ++i) {
            use[i] = i < n;
        }
        Fit f = best_fit(use, n);
        if (n < 6u) {
            return f;
        }
        for (uint8_t k = 0; k < 2u; ++k) {
            uint8_t worst = 0xFF;
            uint32_t worst_d2 = 0;
            for (uint8_t i = 0; i < n; ++i) {
                if (use[i] && residual2(f, i) >= worst_d2) {
                    worst_d2 = residual2(f, i);
                    worst = i;
                }
            }
            if (worst != 0xFFu) {
                use[worst] = false;
            }
        }
        for (uint8_t i = 0; i < targets; ++i) {
            excluded[i] = i < n && !use[i];
        }
        return best_fit(use, n);
    }
    static inline bool excluded[targets];

    static uint32_t isqrt32(uint32_t v) {
        uint32_t r = 0;
        for (uint32_t bit = 1u << 15; bit != 0u; bit >>= 1) {
            const uint32_t t = r | bit;
            if (t * t <= v) {
                r = t;
            }
        }
        return r;
    }

    // ---- the game ---------------------------------------------------------------------
    static uint32_t rnd(uint32_t m) {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        return seed % m;
    }

    /// The next centre, at least 150 px away on one axis, inside the
    /// play area above the sweep's strip.
    static void next_square(bool first) {
        constexpr uint16_t play_h = strip_y - 8;
        uint16_t nx, ny;
        do {
            nx = static_cast<uint16_t>(side / 2u + rnd(ili::lw - side));
            ny = static_cast<uint16_t>(side / 2u + rnd(play_h - side));
        } while (!first && static_cast<uint16_t>(nx > sx ? nx - sx : sx - nx) < 150u &&
                 static_cast<uint16_t>(ny > sy ? ny - sy : sy - ny) < 150u);
        sx = nx;
        sy = ny;
    }

    static void on_tap(Tap t) {
        if (calibrated) {
            return;   // in paint mode a tap is just the first dot
        }
        taps[n_taps] = TapRecord{t.rx, t.ry, sx, sy, t.z};
        ++n_taps;
        int32_t px = 0, py = 0;
        if (n_taps >= 3u) {
            fit = best_fit(n_taps);
            px = mapped_x(fit, t.rx, t.ry);
            py = mapped_y(fit, t.rx, t.ry);
            say("tap ", n_taps, ": raw (", t.rx, ", ", t.ry, ") z ", t.z, " for the square at (", sx, ", ", sy,
                ") -> understood at (", px, ", ", py, ")");
        } else {
            say("tap ", n_taps, ": raw (", t.rx, ", ", t.ry, ") z ", t.z, " for the square at (", sx, ", ", sy, ")");
        }
        // Erase the square and the last cross, place the next square and
        // the new cross.
        enqueue({static_cast<uint16_t>(sx - side / 2u), static_cast<uint16_t>(sy - side / 2u), side, side, 0x000000, false});
        if (cross_shown) {
            cross(cx, cy, 0x000000);
            cross_shown = false;
        }
        if (n_taps < targets) {
            next_square(false);
            enqueue({static_cast<uint16_t>(sx - side / 2u), static_cast<uint16_t>(sy - side / 2u), side, side, 0xFCFCFC, false});
            if (n_taps >= 3u) {
                cross(px, py, 0xFC0000);
                cross_shown = true;
                cx = px;
                cy = py;
            }
        } else {
            print_results();
            say("paint mode: the pen leaves white dots with the fit");
            calibrated = true;
        }
        pump();
    }

    /// The tap table and the fit, whole: at the tenth tap and whenever a
    /// terminal asks (or arrives late).
    static void print_results() {
        for (uint8_t i = 0; i < n_taps; ++i) {
            const TapRecord& r = taps[i];
            say("  tap ", i + 1u, ": raw (", r.rx, ", ", r.ry, ") z ", r.z, " square (", r.sx, ", ", r.sy, ") -> (",
                n_taps >= 3u ? mapped_x(fit, r.rx, r.ry) : 0, ", ", n_taps >= 3u ? mapped_y(fit, r.rx, r.ry) : 0, ")",
                n_taps >= 6u && excluded[i] ? "  left out of the fit" : "");
        }
        if (n_taps < 3u) {
            say("  (no fit before the third tap)");
            return;
        }
        {
            const int32_t ax4 = static_cast<int32_t>(fit.ax * 10000.f), ay4 = static_cast<int32_t>(fit.ay * 10000.f);
            const int32_t bx = static_cast<int32_t>(fit.bx), by = static_cast<int32_t>(fit.by);
            say("RESULT: ", n_taps, " taps; axes ", fit.swap ? "SWAPPED (x from raw y)" : "straight (x from raw x)",
                "; x = u * ", ax4, " / 10000 + ", bx, "; y = v * ", ay4, " / 10000 + ", by, "; error ", fit.rms,
                " px rms, ", fit.worst, " px at worst",
                fit.worst <= 60u ? " - every tap on its square" : " - a tap missed its square");
            say("RESULT: raw at the logical edges: x=0 at u=", static_cast<int32_t>(-fit.bx / fit.ax), ", x=479 at u=",
                static_cast<int32_t>((479.f - fit.bx) / fit.ax), "; y=0 at v=", static_cast<int32_t>(-fit.by / fit.ay),
                ", y=319 at v=", static_cast<int32_t>((319.f - fit.by) / fit.ay));
        }
    }

    /// A new game: the glass cleared, the taps forgotten, the first square.
    static void restart() {
        n_taps = 0;
        calibrated = false;
        cross_shown = false;
        dot_pending = false;
        seed ^= Ticker::millis();
        enqueue({0, 0, ili::lw, ili::lh, 0x000000, false});
        next_square(true);
        enqueue({static_cast<uint16_t>(sx - side / 2u), static_cast<uint16_t>(sy - side / 2u), side, side, 0xFCFCFC, false});
        say("NEW GAME: tap the white square, ten times");
        pump();
    }

    static void cross(int32_t x, int32_t y, uint32_t colour) {
        if (x < 12 || y < 12 || x >= static_cast<int32_t>(ili::lw) - 12 || y >= static_cast<int32_t>(strip_y) - 12) {
            return;
        }
        enqueue({static_cast<uint16_t>(x - 12), static_cast<uint16_t>(y - 1), 25, 3, colour, false});
        enqueue({static_cast<uint16_t>(x - 1), static_cast<uint16_t>(y - 12), 3, 25, colour, false});
    }

    // ---- the FIFO and the bus ------------------------------------------------------------
    static void enqueue(Fill f) {
        if (fifo_count == fifo_depth) {
            ++fills_dropped;
            return;
        }
        fifo[(fifo_head + fifo_count) % fifo_depth] = f;
        ++fifo_count;
    }

    /// Start the next fill if the bus owes nothing: a queued rectangle,
    /// else a dot for the pen, else the sweep band.
    static void pump() {
        if (busy) {
            return;
        }
        if (fifo_count != 0u) {
            cur = fifo[fifo_head];
            fifo_head = static_cast<uint8_t>((fifo_head + 1u) % fifo_depth);
            --fifo_count;
        } else if (dot_pending) {
            dot_pending = false;
            const int32_t x = mapped_x(fit, dot_target.rx, dot_target.ry);
            const int32_t y = mapped_y(fit, dot_target.rx, dot_target.ry);
            if (x < 0 || y < 0 || x >= static_cast<int32_t>(ili::lw) || y >= static_cast<int32_t>(strip_y)) {
                pump();
                return;
            }
            const uint16_t x0 = static_cast<uint16_t>(x > dot / 2 ? x - dot / 2 : 0);
            const uint16_t y0 = static_cast<uint16_t>(y > dot / 2 ? y - dot / 2 : 0);
            cur = Fill{x0, y0, static_cast<uint16_t>(x0 + dot <= ili::lw ? dot : ili::lw - x0),
                       static_cast<uint16_t>(y0 + dot <= strip_y ? dot : strip_y - y0), 0xFCFCFC, false};
            ++dots_painted;
        } else {
            cur = Fill{0, band_y, ili::lw, band_rows, hue_colour(hue), false};
            band_y = static_cast<uint16_t>(band_y + band_rows);
            if (band_y + band_rows > ili::lh) {
                band_y = strip_y;
            }
            ++hue;
            ++bands_painted;
        }
        busy = true;
        step = 0;
        row = 0;
        ili::window_params(cur.x, cur.y, cur.w, cur.h, cas, pas);
        for (uint16_t i = 0; i < cur.w; ++i) {
            ili::put_pixel(row_buf, i, cur.colour);
        }
        send(ili::caset, cas, 4);
    }

    static void advance(bool ok) {
        if (!ok) {
            ++failed;
        }
        if (step == 0u) {
            step = 1;
            send(ili::paset, pas, 4);
            return;
        }
        if (step == 1u) {
            step = 2;
            if (cur.verify) {
                read(static_cast<uint16_t>(cur.w * 3u + 4u));
            } else {
                send(ili::ramwr, row_buf, static_cast<uint16_t>(cur.w * 3u));
            }
            return;
        }
        if (cur.verify) {
            // One dummy byte, then the pixels (the probe's finding).
            uint16_t good = 0;
            uint8_t want[3];
            ili::put_pixel(want, 0, cur.colour);
            for (uint16_t i = 0; i < cur.w; ++i) {
                const uint8_t* px = read_buf + 1u + i * 3u;
                if ((px[0] & 0xFCu) == want[0] && (px[1] & 0xFCu) == want[1] && (px[2] & 0xFCu) == want[2]) {
                    ++good;
                }
            }
            verified_ok += good;
            verified_bad += static_cast<uint32_t>(cur.w - good);
            for (uint8_t i = 0; i < 8u; ++i) {
                verify_head[i] = read_buf[i];
            }
            if (verify_count < 8u) {
                verifies[verify_count] = VerifyRecord{cur.x, cur.y, good, cur.w,
                                                      {read_buf[0], read_buf[1], read_buf[2], read_buf[3]}};
                ++verify_count;
            }
            say("verify: ", good, " of ", cur.w, " pixels of the square's row read back white; first bytes ",
                read_buf[0], " ", read_buf[1], " ", read_buf[2], " ", read_buf[3]);
            busy = false;
            pump();
            return;
        }
        ++row;
        ++rows_painted;
        if (row < cur.h) {
            send(ili::ramwr_continue, row_buf, static_cast<uint16_t>(cur.w * 3u));
            return;
        }
        busy = false;
        pump();
    }

    static void send(uint8_t c, const uint8_t* data, uint16_t n) {
        cmd = c;
        Host::Request r{};
        r.cs = Cs::ref();
        r.dc = Dc::ref();
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&cmd));
        r.cmd_len = 1;
        r.tx = lend<Lease::reply>(data);
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
        r.len = n;
        r.reply = reply_to<Painter, SpiDone>();
        r.clock = panel_rate;
        r.mode = SpiMode::mode0;
        r.polled = false;
        post<Bus>(r);
    }

    /// RAMRD of `n` bytes into read_buf, at the probe's read rate.
    static void read(uint16_t n) {
        cmd = 0x2E;
        Host::Request r{};
        r.cs = Cs::ref();
        r.dc = Dc::ref();
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&cmd));
        r.cmd_len = 1;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(read_buf));
        r.len = n;
        r.reply = reply_to<Painter, SpiDone>();
        r.clock = SpiClock::div32;   // 3 MHz
        r.mode = SpiMode::mode0;
        r.polled = false;
        post<Bus>(r);
    }

    /// A colour wheel of 96 steps, six bits a channel.
    static uint32_t hue_colour(uint8_t h) {
        const uint8_t seg = static_cast<uint8_t>((h / 16u) % 6u);
        const uint8_t f = static_cast<uint8_t>((h % 16u) * 16u);
        const uint8_t up = f, down = static_cast<uint8_t>(240u - f);
        uint8_t r = 0, g = 0, b = 0;
        switch (seg) {
            case 0: r = 240; g = up; break;
            case 1: r = down; g = 240; break;
            case 2: g = 240; b = up; break;
            case 3: g = down; b = 240; break;
            case 4: b = 240; r = up; break;
            default: b = down; r = 240; break;
        }
        return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | b;
    }
};

// ---- the touch client ----------------------------------------------------------
struct Touch : Fsm<Touch, Poll, Report, SpiDone> {
    static inline EventQueue<Event, 4, P> queue;
    static inline TimeEvent<P, Touch, Poll> cadence{Poll{}};
    static inline TimeEvent<P, Touch, Report> reporter{Report{}};

    static constexpr uint16_t poll_ms = 20;
    static constexpr uint16_t z_threshold = 100;
    static constexpr uint8_t burst_n = 8;
    static constexpr uint8_t burst_skip = 2;
    static constexpr uint8_t burst_bytes = burst_n * 3;

    static inline uint8_t phase = 0;
    static inline uint8_t tx[burst_bytes];
    static inline uint8_t rx[burst_bytes];
    static inline uint16_t z = 0, xr = 0, yr = 0;
    static inline bool was_pressed = false;
    static inline bool in_flight = false;
    static inline uint8_t idle_run = 0;
    static inline uint32_t polls = 0, conversions = 0, samples = 0, strokes = 0, irq_low_polls = 0;

    static void init() {
        Tcs::output(true);
        Tirq::input(PinPull::up);
        start(&polling);
    }

    static Status polling(const Event& e) {
        return match(e,
            [](Entry) {
                cadence.arm_every(ticks_from_ms<P>(poll_ms));
                reporter.arm_every(ticks_from_ms<P>(2000));
                return handled();
            },
            [](Poll) {
                ++polls;
                if (!Tirq::read()) {
                    ++irq_low_polls;
                }
                // The console's keys, and the results once for a terminal
                // that arrives after the game.
                uint8_t key = 0;
                while (Serial::read_byte(key)) {
                    if (key == 'g' || key == 'G') {
                        post<Painter>(Restart{});
                    } else if (key == 'r' || key == 'R') {
                        post<Painter>(ShowResults{});
                    }
                }
                const bool terminal = Serial::configured() && Serial::dtr();
                if (terminal && !terminal_seen) {
                    say("spi_duo: keys g = new game, r = the taps and the fit");
                    if (Painter::n_taps != 0u) {
                        post<Painter>(ShowResults{});
                    }
                }
                terminal_seen = terminal;
                if (in_flight || !touch_enabled) {
                    return handled();
                }
                phase = 0;
                // Z1 with PD = 00: powered down between conversions and
                // PENIRQ ENABLED (PD = 01 would hold the line high).
                convert(0xB0);
                return handled();
            },
            [](SpiDone d) {
                in_flight = false;
                if (d.status != spi_ok) {
                    return handled();
                }
                ++conversions;
                switch (phase) {
                    case 0:
                        z = value_at(0);
                        if (z > z_threshold) {
                            phase = 1;
                            burst(0xD1, 0xD1);   // X burst, kept powered
                        } else {
                            if (idle_run < 255u) {
                                ++idle_run;
                            }
                            pressed_run = 0;
                            pen_up_if_needed();
                        }
                        break;
                    case 1:
                        xr = trimmed_mean();
                        phase = 2;
                        burst(0x91, 0x90);       // Y burst, powered down last
                        break;
                    default:
                        yr = trimmed_mean();
                        on_sample();
                        break;
                }
                return handled();
            },
            [](Report) {
                say("bus: rows ", Painter::rows_painted, " bands ", Painter::bands_painted, " dots ",
                    Painter::dots_painted, " fills dropped ", Painter::fills_dropped, " failed ", Painter::failed,
                    " | touch: polls ", polls,
                    " penirq low ", irq_low_polls, " conversions ", conversions, " samples ", samples, " strokes ",
                    strokes, " | rejected ", Bus::rejected_count(), " penirq=", Tirq::read() ? "high" : "LOW",
                    " last z1=", z, " taps ", Painter::n_taps, " | bring-up statuses ", ili::bring_up_status[0], " ",
                    ili::bring_up_status[1], " ", ili::bring_up_status[2], " ", ili::bring_up_status[3], " verify ok/bad ",
                    Painter::verified_ok, "/", Painter::verified_bad, " head ", Painter::verify_head[0], " ",
                    Painter::verify_head[1], " ", Painter::verify_head[2], " ", Painter::verify_head[3], " ",
                    Painter::verify_head[4], " host ", host_ok ? "ok" : "FAILED");
                for (uint8_t i = 0; i < Painter::verify_count; ++i) {
                    const Painter::VerifyRecord& v = Painter::verifies[i];
                    say("  verify at (", v.x, ",", v.y, "): ", v.good, " of ", v.w, " pixels as expected; head ", v.head[0],
                        " ", v.head[1], " ", v.head[2], " ", v.head[3]);
                }
                return handled();
            },
            [](auto) { return unhandled(); });
    }

private:
    static void pen_up_if_needed() {
        if (was_pressed) {
            was_pressed = false;
            post<Painter>(PenUp{});
        }
    }

    /// A stroke begins after three idle polls, and its TAP is the FOURTH
    /// pressed sample in a row (80 ms in): the first samples of a stroke
    /// are taken while the finger is still landing and their coordinates
    /// lie far from where it settles - two of ten taps of the first game
    /// were such, 180 px off. Never in the first half second.
    static inline uint8_t pressed_run = 0;
    static inline bool terminal_seen = false;
    static void on_sample() {
        ++samples;
        if (pressed_run < 255u) {
            ++pressed_run;
        }
        if (!was_pressed && idle_run >= 3u && pressed_run >= 4u && polls > 25u) {
            was_pressed = true;
            ++strokes;
            post<Painter>(Tap{xr, yr, z});
        }
        if (was_pressed) {
            idle_run = 0;
            post<Painter>(Sample{xr, yr});
        }
    }

    static uint16_t value_at(uint8_t i) {
        return static_cast<uint16_t>((load_be16(rx + 3u * i + 1u) >> 3) & 0x0FFFu);
    }

    static uint16_t trimmed_mean() {
        uint16_t lo = 0xFFFF, hi = 0;
        uint32_t sum = 0;
        for (uint8_t i = burst_skip; i < burst_n; ++i) {
            const uint16_t v = value_at(i);
            sum += v;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        return static_cast<uint16_t>((sum - lo - hi) / (burst_n - burst_skip - 2u));
    }

    static void convert(uint8_t ctrl) {
        tx[0] = ctrl;
        tx[1] = 0;
        tx[2] = 0;
        post_xfer(3);
    }

    static void burst(uint8_t ctrl, uint8_t last_ctrl) {
        for (uint8_t i = 0; i < burst_n; ++i) {
            tx[3u * i] = (i == burst_n - 1u) ? last_ctrl : ctrl;
            tx[3u * i + 1u] = 0;
            tx[3u * i + 2u] = 0;
        }
        post_xfer(burst_bytes);
    }

    static void post_xfer(uint16_t n) {
        Host::Request r{};
        r.cs = Tcs::ref();
        r.dc = PinRef{};   // the XPT2046 has no such line
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
        r.cmd_len = 0;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
        r.rx = lend<Lease::reply>(rx);
        r.len = n;
        r.reply = reply_to<Touch, SpiDone>();
        r.clock = touch_rate;
        r.mode = SpiMode::mode0;
        r.polled = false;
        in_flight = true;
        post<Bus>(r);
    }
};

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void SPI1_IRQHandler() {
    if (Host::isr() && bus_live) {
        brio::post<Bus>(brio::TransferDone{Host::status()});
    }
}
extern "C" void DMA2_Stream3_IRQHandler() {
    if (Host::dma_isr() && bus_live) {
        brio::post<Bus>(brio::TransferDone{Host::status()});
    }
}
extern "C" void DMA2_Stream0_IRQHandler() {
    if (Host::dma_isr() && bus_live) {
        brio::post<Bus>(brio::TransferDone{Host::status()});
    }
}
extern "C" void OTG_FS_IRQHandler() { Device::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    (void)SysClock::init();
    (void)brio::Ticker::init(clock);
    Led::output(true);
    brio::enable_interrupts();

    Cs::output(true);
    Rst::output(true);
    Dc::output(true);
    Tcs::output(true);
    const bool usb_ok = Usb::init(clock);
    if (usb_ok) {
        Device::start();
    }
    (void)BootHost::init(clock);
    ili::bring_up();
    BootHost::release();
    host_ok = Host::init(clock);
    Host::sck_speed(PinSpeed::medium);
    Host::mosi_speed(PinSpeed::medium);
    Led::clear();

    // The banner waits for a terminal, at most two seconds: the
    // experiment runs with or without one.
    const uint32_t t0 = brio::Ticker::millis();
    while ((!Serial::configured() || !Serial::dtr()) && brio::Ticker::millis() - t0 < 2000u) {
    }
    say("spi_duo: ILI9481 + XPT2046 on SPI1 through one SpiBus; rows at 12 MHz, touch at 1.5 MHz; ",
        "touch the glass");

    bus_live = true;
    brio::Tenuto<P, Painter, Touch, Bus>::run();
}
