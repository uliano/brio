// display_probe - empirical identification of the mystery 2.4" display:
// hardware reset, minimal ILI9341 init sequence (timed by time events -
// the 120 ms sleep-out wait is mandatory), then fill the whole screen
// RED, one 240-pixel row per SPI transaction (320 chained requests -
// a 150 KB framebuffer does not fit in 16 KB of RAM, and the ILI9341
// memory write happily continues across CS windows).
//
// Verdict on the panel: uniform RED (or BLUE: just the MADCTL BGR bit) =
// ILI9341 family confirmed. Garbage / nothing = next suspect (ST7789).
// Progress and timing are reported on the serial console.
//
// Wiring (README bench map): CS=PD0, RS/DC=PD1, RST=PD2, SDA<-PA4,
// CLK<-PA6, VCC=5V with the module's J1 open.

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrdx/clock.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/spi.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/uart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/spi_ao.hpp"

using P = brio::AvrPlatform;

namespace {

template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

using SpiHw = brio::Spi<0>;
using SpiBus = brio::SpiAo<SpiHw, P>;

using CsPin = brio::Pin<'D', 0>;
using DcPin = brio::Pin<'D', 1>;
using RstPin = brio::Pin<'D', 2>;

constexpr uint16_t width = 240;
constexpr uint16_t height = 320;

// Minimal ILI9341 bring-up. wait_ms after each step; SLPOUT needs 120+.
struct InitStep {
    uint8_t cmd;
    uint8_t wait_ms;
    uint8_t len;
    uint8_t data[4];
};
constexpr InitStep init_steps[] = {
    {0x01, 150, 0, {}},                    // SWRESET (belt and braces)
    {0x11, 150, 0, {}},                    // SLPOUT + mandatory wait
    {0x3A, 5, 1, {0x55}},                  // COLMOD: 16-bit RGB565
    {0x36, 5, 1, {0x48}},                  // MADCTL: MX | BGR (typical)
    {0x29, 25, 0, {}},                     // DISPON
    {0x2A, 1, 4, {0, 0, 0, width - 1}},    // CASET: columns 0..239
    {0x2B, 1, 4, {0, 0, 0x01, 0x3F}},      // PASET: rows 0..319
    {0x2C, 1, 0, {}},                      // RAMWR: pixel stream follows
};
constexpr uint8_t init_count = sizeof(init_steps) / sizeof(init_steps[0]);

struct Tick {};

struct ProbeAo : brio::Fsm<ProbeAo, Tick, brio::SpiDone> {
    static inline brio::EventQueue<Event, 3, P> queue;
    static inline brio::TimeEvent<P, ProbeAo, Tick> timer{Tick{}};

    static inline uint8_t phase = 0;       // within `resetting`
    static inline uint8_t step = 0;        // within `initing`
    static inline uint16_t row = 0;        // within `filling`
    static inline uint8_t cur_cmd = 0;     // storage for the in-flight cmd
    static inline uint8_t row_buf[width * 2];
    static inline uint32_t t0 = 0;

    static void init() {
        CsPin::set();
        DcPin::set();
        RstPin::set();
        CsPin::output();
        DcPin::output();
        RstPin::output();
        brio::Pin<'D', 3>::set();          // deselect the MCP3550: shared bus
        brio::Pin<'D', 3>::output();
        start(&resetting);
    }

    // -- hardware reset: RST low 5 ms, high, settle 150 ms ---------------
    static Status resetting(const Event& e) {
        return std::visit(overloaded{
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
                return transition(&initing);
            },
            [](auto) { return unhandled(); },
        }, e);
    }

    // -- init table: one command per SPI request, timed gaps -------------
    static Status initing(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                step = 0;
                send_step();
                return handled();
            },
            [](brio::SpiDone) {
                timer.arm(brio::ticks_from_ms<P>(init_steps[step].wait_ms));
                return handled();
            },
            [](Tick) {
                if (++step < init_count) {
                    send_step();
                    return handled();
                }
                return transition(&filling);
            },
            [](auto) { return unhandled(); },
        }, e);
    }

    // -- fill: 320 rows of RED, chained on each completion ---------------
    static Status filling(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                for (uint16_t i = 0; i < width; ++i) {
                    row_buf[2 * i] = 0xF8;         // RGB565 red, high byte
                    row_buf[2 * i + 1] = 0x00;
                }
                row = 0;
                t0 = brio::Ticker::millis();
                send_row();
                return handled();
            },
            [](brio::SpiDone) {
                if (++row < height) {
                    send_row();
                    return handled();
                }
                fill_ms = brio::Ticker::millis() - t0;
                return transition(&done);
            },
            [](auto) { return unhandled(); },
        }, e);
    }

    // -- verdict repeated so a late-opened monitor still catches it ------
    static Status done(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                timer.arm_every(brio::ticks_from_ms<P>(2000));
                return handled();
            },
            [](Tick) {
                brio::print(serial, "screen filled in ", fill_ms,
                            " ms - is it RED? (RED/BLUE = ILI9341 family, "
                            "garbage = next suspect)", brio::crlf);
                return handled();
            },
            [](auto) { return unhandled(); },
        }, e);
    }

    static inline uint32_t fill_ms = 0;

private:
    static void send_step() {
        const InitStep& s = init_steps[step];
        cur_cmd = s.cmd;
        brio::print(serial, "cmd ", brio::hex(s.cmd), brio::crlf);
        brio::post<SpiBus>(SpiHw::Request{
            CsPin::ref(), DcPin::ref(),
            &cur_cmd, 1,
            s.data, nullptr, s.len,
            brio::reply_to<ProbeAo, brio::SpiDone>()});
    }

    static void send_row() {
        brio::post<SpiBus>(SpiHw::Request{
            CsPin::ref(), DcPin::ref(),
            nullptr, 0,                        // RAMWR continuation: data only
            row_buf, nullptr, sizeof(row_buf),
            brio::reply_to<ProbeAo, brio::SpiDone>()});
    }
};

} // namespace

// ---- target glue ------------------------------------------------------------
ISR(SPI0_INT_vect) {
    if (SpiHw::isr()) {
        brio::post<SpiBus>(brio::TransferDone{brio::spi_ok});
    }
}
ISR(USART2_RXC_vect) { Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    brio::init_clock_24mhz();
    Serial::init(460800);
    SpiHw::init(brio::SpiClock::div4);         // 6 MHz, well within ILI9341 spec
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf, "display probe: ILI9341 hypothesis",
                brio::crlf);

    brio::Kernel<P, ProbeAo, SpiBus>::run();
}
