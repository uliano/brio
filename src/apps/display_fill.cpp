// display_fill - first pixels on the 3.5" ILI9481 module (HST035003-A,
// identified by display_id: DEVCODE 02 04 94 81). Full-screen solid
// fill cycling red -> green -> blue every couple of seconds: if the
// panel shows the colors, the whole write path (commands + GRAM) is
// proven and the display is bench-ready.
//
// ILI9481 facts used here:
//   - native 320 (columns) x 480 (pages), MADCTL default 0x00
//   - SPI (DBI type C) supports ONLY the 18-bit pixel format
//     (COLMOD 0x66, the reset default): 3 bytes/pixel, each color in
//     the 6 MSBs of its byte (0xFC = full)
//   - DCS windowing: CASET 0x2A / PASET 0x2B (params travel in the
//     data phase, DC high - exactly our descriptor's phase 2), then
//     RAMWR 0x2C + continuation writes 0x3C
//
// One request per row (320 px * 3 = 960 bytes <= uint16_t len); a full
// frame is 480 requests. SPI at 6 MHz (div4): a fill takes ~1-2 s,
// dominated by the per-byte ISR pump - fine for a probe.
//
// Wiring per README bench map (3.3 V logic!), BL tied high.

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

using P = brio::AvrPlatform;

// The clock: the ONE truth about CLK_PER for every driver of this
// target (avrdx/clock.hpp). 24 MHz crystal on PA0/PA1, OSCHF fallback at
// the same rate; `clock` is an empty tag passed to driver inits.
using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

using SpiHw = brio::SpiHost<0>;
using Bus = brio::SpiBus<SpiHw, P>;

using CsPin = brio::Pin<'D', 0>;
using DcPin = brio::Pin<'D', 1>;
using RstPin = brio::Pin<'D', 2>;

struct Tick {};

constexpr uint16_t width = 320;   // columns (CASET)
constexpr uint16_t height = 480;  // pages (PASET)
constexpr uint16_t row_bytes = width * 3;

struct Rgb {
    uint8_t r, g, b;              // 6 significant MSBs each
    const char* name;
};
constexpr Rgb colors[] = {
    {0xFC, 0x00, 0x00, "RED"},
    {0x00, 0xFC, 0x00, "GREEN"},
    {0x00, 0x00, 0xFC, "BLUE"},
};
constexpr uint8_t color_count = sizeof(colors) / sizeof(colors[0]);

struct Filler : brio::Fsm<Filler, Tick, brio::SpiDone> {
    static inline brio::EventQueue<Event, 3, P> queue;
    static inline brio::TimeEvent<P, Filler, Tick> timer{Tick{}};

    static inline uint8_t phase = 0;
    static inline uint8_t color = 0;
    static inline uint16_t row = 0;
    static inline uint8_t cur_cmd = 0;
    static inline uint8_t params[4];
    static inline uint8_t row_buf[row_bytes];

    static void init() {
        CsPin::set();  CsPin::output();
        DcPin::set();  DcPin::output();
        RstPin::set(); RstPin::output();
        brio::Pin<'D', 3>::set();              // deselect the MCP3550
        brio::Pin<'D', 3>::output();
        start(&resetting);
    }

    static Status resetting(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                phase = 0;
                RstPin::clear();
                timer.arm(brio::ticks_from_ms<P>(5));
                return handled();
            },
            [](Tick) {
                if (phase++ == 0) {
                    RstPin::set();
                    timer.arm(brio::ticks_from_ms<P>(150));
                    return handled();
                }
                return transition(&waking);
            },
            [](auto) { return unhandled(); }
        );
    }

    // SLPOUT, wait 150 ms, DISPON, wait 25 ms, INVON -> framing.
    // INVON: these ILI9481 panels are wired with inverted polarity and
    // show complementary colors without it (the classic 9481 quirk).
    static Status waking(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                phase = 0;
                send(0x11, nullptr, 0);        // SLPOUT
                return handled();
            },
            [](brio::SpiDone) {
                if (phase == 2) {
                    return transition(&framing);
                }
                timer.arm(brio::ticks_from_ms<P>(phase == 0 ? 150 : 25));
                return handled();
            },
            [](Tick) {
                ++phase;
                send(phase == 1 ? 0x29 : 0x21, nullptr, 0);  // DISPON, INVON
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }

    // full-screen window: CASET then PASET
    static Status framing(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                phase = 0;
                set_window(0x2A, width - 1);   // CASET 0..319
                return handled();
            },
            [](brio::SpiDone) {
                if (phase++ == 0) {
                    set_window(0x2B, height - 1);  // PASET 0..479
                    return handled();
                }
                return transition(&filling);
            },
            [](auto) { return unhandled(); }
        );
    }

    static Status filling(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                const Rgb& c = colors[color];
                for (uint16_t i = 0; i < row_bytes; i += 3) {
                    row_buf[i] = c.r;
                    row_buf[i + 1] = c.g;
                    row_buf[i + 2] = c.b;
                }
                row = 0;
                send(0x2C, row_buf, row_bytes);    // RAMWR, first row
                return handled();
            },
            [](brio::SpiDone) {
                if (++row < height) {
                    send(0x3C, row_buf, row_bytes);  // write continue
                    return handled();
                }
                brio::print(serial, "filled ", colors[color].name,
                            brio::crlf);
                timer.arm(brio::ticks_from_ms<P>(1500));
                return handled();
            },
            [](Tick) {
                color = static_cast<uint8_t>((color + 1) % color_count);
                return transition(&filling);   // re-Entry rebuilds the row
            },
            [](auto) { return unhandled(); }
        );
    }

private:
    static void send(uint8_t c, const uint8_t* data, uint16_t n) {
        cur_cmd = c;
        brio::post<Bus>(SpiHw::Request{
            CsPin::ref(), DcPin::ref(),
            brio::lend<brio::Lease::reply>(&cur_cmd), 1,
            brio::lend<brio::Lease::reply>(data), {}, n,
            brio::reply_to<Filler, brio::SpiDone>(),
            brio::SpiClock::div4,              // ILI9481 happily at 6 MHz
            brio::SpiMode::mode0, true});             // polled: bulk at wire speed
    }

    static void set_window(uint8_t c, uint16_t last) {
        params[0] = 0;
        params[1] = 0;
        params[2] = static_cast<uint8_t>(last >> 8);
        params[3] = static_cast<uint8_t>(last & 0xFF);
        send(c, params, 4);
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
    SpiHw::init(clock);                             // clock/mode travel per-request
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf,
                "ILI9481 full-screen fill: red/green/blue cycle",
                brio::crlf);

    brio::Kernel<P, Filler, Bus>::run();
}
