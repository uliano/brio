// display_inv - naked-eye diagnostic for the mystery display's COMMAND
// path: hardware reset, SLPOUT + DISPON, then toggle display inversion
// (INVON 0x21 / INVOFF 0x20 - parameterless MIPI-DCS commands) twice a
// second. A woken panel visibly pulses white <-> black even with no
// pixel ever written.
//
//   pulsing  -> command path (CS/DC/RST/SDA/CLK wiring + controller) OK:
//               any remaining problem is in the pixel/data phase
//   nothing  -> commands not getting in: check CS->PD0, RS->PD1,
//               RST->PD2 ordering and the COMMON GROUND first
//
// Wiring per README bench map. Serial reports every toggle.

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

struct Tick {};

struct InvAo : brio::Fsm<InvAo, Tick, brio::SpiDone> {
    static inline brio::EventQueue<Event, 3, P> queue;
    static inline brio::TimeEvent<P, InvAo, Tick> timer{Tick{}};

    static inline uint8_t phase = 0;
    static inline uint8_t cur_cmd = 0;
    static inline bool inverted = false;

    static void init() {
        CsPin::set();  CsPin::output();
        DcPin::set();  DcPin::output();
        RstPin::set(); RstPin::output();
        brio::Pin<'D', 3>::set();              // deselect the MCP3550
        brio::Pin<'D', 3>::output();
        start(&resetting);
    }

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
                return transition(&waking);
            },
            [](auto) { return unhandled(); },
        }, e);
    }

    // SLPOUT, wait 150 ms, DISPON, wait 25 ms -> blink
    static Status waking(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                phase = 0;
                send_cmd(0x11);                // SLPOUT
                return handled();
            },
            [](brio::SpiDone) {
                timer.arm(brio::ticks_from_ms<P>(phase == 0 ? 150 : 25));
                return handled();
            },
            [](Tick) {
                if (phase++ == 0) {
                    send_cmd(0x29);            // DISPON
                    return handled();
                }
                return transition(&blinking);
            },
            [](auto) { return unhandled(); },
        }, e);
    }

    static Status blinking(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                timer.arm_every(brio::ticks_from_ms<P>(500));
                brio::print(serial, "blinking INVON/INVOFF - does the panel pulse?",
                            brio::crlf);
                return handled();
            },
            [](Tick) {
                inverted = !inverted;
                send_cmd(inverted ? 0x21 : 0x20);
                brio::print(serial, inverted ? "INVON" : "INVOFF", brio::crlf);
                return handled();
            },
            [](brio::SpiDone) { return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }

private:
    static void send_cmd(uint8_t c) {
        cur_cmd = c;
        brio::post<SpiBus>(SpiHw::Request{
            CsPin::ref(), DcPin::ref(),
            &cur_cmd, 1,
            nullptr, nullptr, 0,
            brio::reply_to<InvAo, brio::SpiDone>()});
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
    SpiHw::init(brio::SpiClock::div16);        // 1.5 MHz, canonical mode 0
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf, "display inversion blink probe", brio::crlf);

    brio::Kernel<P, InvAo, SpiBus>::run();
}
