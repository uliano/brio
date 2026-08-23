// spi_duo - TWO devices, ONE SPI bus, arbitration by active objects:
// the whole point of the SpiBus design, finally on real silicon.
//
//   Filler  - the ILI9481 display client: continuously repaints the
//             320x480 panel (480 requests of 960 bytes per frame, at
//             6 MHz) - the bus hog.
//   Touch   - the XPT2046 touch client (same module, own chip select):
//             polls at 20 Hz with short 3-byte conversions capped at
//             1.5 MHz - the latency-sensitive little guy.
//
// Neither client knows the other exists. Both post Spi<0>::Request
// events to the same SpiBus; the AO's queue + pending FIFO serialize
// them, each request carries its OWN clock rate (per-transaction
// engine reconfiguration), and every reply finds its way home through
// the ReplyTo capsule. The two clients also exercise BOTH completion
// styles: display rows go POLLED (bulk at wire speed, ~2.2 ms per
// 960-byte row, completed synchronously inside start()), the touch
// conversions ride the per-byte ISR pump. A touch request at worst
// waits out the row in flight - invisible at 20 Hz.
//
// Touch feedback on the panel itself: the fill color follows the
// touched screen half - top half = next color, bottom half = previous.
// The serial console streams touch samples and per-frame stats.
//
// Extra wiring on top of the README bench map (module touch section
// joins the shared bus):
//   T_CLK -> PA6 (SCK)     T_DIN -> PA4 (MOSI)
//   T_DO  -> PA5 (MISO)    T_CS  -> PD5        (PEN not used: polling)
//
// XPT2046 conversation (mode 0, <= 2.5 MHz): send a control byte, then
// two dummies while the 12-bit result shifts out: value = ((b1<<8 |
// b2) >> 3) & 0xFFF. Z1 (ctrl 0xB1, PD1:0=01 keeps VREF between
// samples) gauges pressure: near 0 = no touch. When pressed, X (0xD1)
// and Y (0x91) are read back-to-back; the LAST conversion uses 0x90
// (PD=00) to power the chip back down.

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

// display client pins (README bench map)
using CsPin = brio::Pin<'D', 0>;
using DcPin = brio::Pin<'D', 1>;
using RstPin = brio::Pin<'D', 2>;
// touch client pin
using TcsPin = brio::Pin<'D', 5>;

// ---- shared bench state (main context only) ---------------------------------
// Touch publishes the latest pressed position; Filler reads it when
// picking the next frame's color. Plain statics are enough: both AOs
// run in the same RTC loop.
inline int8_t color_step = 1;      // +1 = forward, -1 = backward
inline uint16_t touch_count = 0;   // samples seen pressed

// ============================ display client ================================

struct Tick {};

constexpr uint16_t width = 320;
constexpr uint16_t height = 480;
constexpr uint16_t row_bytes = width * 3;

struct Rgb {
    uint8_t r, g, b;
    const char* name;
};
constexpr Rgb colors[] = {
    {0xFC, 0x00, 0x00, "RED"},
    {0xFC, 0xFC, 0x00, "YELLOW"},
    {0x00, 0xFC, 0x00, "GREEN"},
    {0x00, 0xFC, 0xFC, "CYAN"},
    {0x00, 0x00, 0xFC, "BLUE"},
    {0xFC, 0x00, 0xFC, "MAGENTA"},
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

    // SLPOUT, wait 150 ms, DISPON, wait 25 ms, INVON (9481 panel
    // polarity quirk) -> framing
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

    static Status framing(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                phase = 0;
                set_window(0x2A, width - 1);
                return handled();
            },
            [](brio::SpiDone) {
                if (phase++ == 0) {
                    set_window(0x2B, height - 1);
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
                send(0x2C, row_buf, row_bytes);
                return handled();
            },
            [](brio::SpiDone d) {
                if (d.status != brio::spi_ok) {  // rejected: retry this row
                    send(row == 0 ? 0x2C : 0x3C, row_buf, row_bytes);
                    return handled();
                }
                if (++row < height) {
                    send(0x3C, row_buf, row_bytes);
                    return handled();
                }
                brio::print(serial, "frame ", colors[color].name,
                            " done, touches so far ", touch_count,
                            ", bus rejects ", Bus::rejected_count(),
                            brio::crlf);
                timer.arm(brio::ticks_from_ms<P>(700));
                return handled();
            },
            [](Tick) {
                color = static_cast<uint8_t>(
                    (color + color_count + color_step) % color_count);
                return transition(&filling);
            },
            [](auto) { return unhandled(); }
        );
    }

private:
    static void send(uint8_t c, const uint8_t* data, uint16_t n) {
        cur_cmd = c;
        brio::post<Bus>(SpiHw::Request{
            CsPin::ref(), DcPin::ref(),
            &cur_cmd, 1,
            data, nullptr, n,
            brio::reply_to<Filler, brio::SpiDone>(),
            brio::SpiClock::div4,              // 6 MHz for the panel
            SPI_MODE_0_gc, true});             // polled: bulk at wire speed
    }

    static void set_window(uint8_t c, uint16_t last) {
        params[0] = 0;
        params[1] = 0;
        params[2] = static_cast<uint8_t>(last >> 8);
        params[3] = static_cast<uint8_t>(last & 0xFF);
        send(c, params, 4);
    }
};

// ============================= touch client =================================

struct Poll {};

struct Touch : brio::Fsm<Touch, Poll, brio::SpiDone> {
    static inline brio::EventQueue<Event, 3, P> queue;
    static inline brio::TimeEvent<P, Touch, Poll> cadence{Poll{}};

    static constexpr uint16_t z_threshold = 100;

    static inline uint8_t phase = 0;
    static inline uint8_t tx[3];
    static inline uint8_t rx[3];
    static inline uint16_t z = 0;
    static inline uint16_t x = 0;
    static inline uint16_t y = 0;
    static inline bool was_pressed = false;
    static inline uint8_t debug_div = 0;      // 1 Hz raw-Z heartbeat

    static void init() {
        TcsPin::set();
        TcsPin::output();
        start(&polling);
    }

    static Status polling(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                cadence.arm_every(brio::ticks_from_ms<P>(50));
                return handled();
            },
            [](Poll) {
                phase = 0;
                convert(0xB1);                 // Z1, keep powered
                return handled();
            },
            [](brio::SpiDone d) {
                if (d.status != brio::spi_ok) {
                    return handled();          // lost this 50 ms slot, fine
                }
                const uint16_t v = value();
                switch (phase) {
                    case 0:
                        z = v;
                        if (++debug_div >= 20) {   // once a second: raw look
                            debug_div = 0;
                            brio::print(serial, "z1 raw ", z, " bytes ",
                                        brio::hex(rx[0]), " ", brio::hex(rx[1]),
                                        " ", brio::hex(rx[2]), brio::crlf);
                        }
                        if (z > z_threshold) {
                            phase = 1;
                            convert(0xD1);     // X, keep powered
                        } else {
                            if (was_pressed) {
                                brio::print(serial, "touch up", brio::crlf);
                            }
                            was_pressed = false;
                        }
                        break;
                    case 1:
                        x = v;
                        phase = 2;
                        convert(0x91);         // Y, keep powered
                        break;
                    case 2:
                        y = v;
                        phase = 3;
                        convert(0x90);         // Y again + power down
                        break;
                    default:
                        y = v;
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
        ++touch_count;
        if (!was_pressed) {
            // steer the fill: touch in the top half runs the palette
            // forward, bottom half backward (Y grows toward the bottom)
            color_step = (y < 2048) ? 1 : -1;
            brio::print(serial, "touch down x=", x, " y=", y, " z=", z,
                        " -> step ", color_step, brio::crlf);
        }
        was_pressed = true;
    }

    static uint16_t value() {
        // 12-bit result rides a big-endian 16-bit word, MSB-justified
        return static_cast<uint16_t>((brio::load_be16(rx + 1) >> 3) & 0x0FFF);
    }

    static void convert(uint8_t ctrl) {
        tx[0] = ctrl;
        tx[1] = 0;
        tx[2] = 0;
        brio::post<Bus>(SpiHw::Request{
            TcsPin::ref(), {},                 // no DC on the XPT2046
            nullptr, 0,
            tx, rx, 3,
            brio::reply_to<Touch, brio::SpiDone>(),
            brio::SpiClock::div16});           // 1.5 MHz, XPT2046 limit
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
                "spi_duo: ILI9481 fill + XPT2046 touch on one arbitrated bus",
                brio::crlf,
                "touch the screen: top half = next color, bottom = previous",
                brio::crlf);

    brio::Kernel<P, Bus, Touch, Filler>::run();
}
