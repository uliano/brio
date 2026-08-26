// display_id - the discriminating probe for the mystery display: READ
// from the controller instead of writing blind. Requires the module's
// SDO pin wired to PA5 (MISO). After the usual reset + SLPOUT + DISPON
// dance it polls the MIPI-DCS read registers once per second and prints
// the raw answers:
//
//   0x0A RDDPM    -> 9C ...   (power mode: booster on, sleep out, ...)
//   0x04 RDDID    -> XX II II II
//   0xD3 ID4      -> XX 00 93 41   (ILI9341 signature)
//
//   stable non-00/non-FF bytes -> controller ALIVE and talking: the
//       whole electrical path (buffer U2 included) is proven, a frozen
//       white panel is then a panel/flex/COG verdict
//   all 00 or all FF           -> controller not answering: problem is
//       upstream of the COG (or SDO not wired/not present)
//
// Multi-byte DCS reads have a dummy clock cycle after the command byte:
// the first byte is garbage and data may arrive shifted by one bit.
// Raw bytes are printed as received - interpretation is done at the
// bench, aliveness is what matters here.
//
// Wiring per README bench map + module SDO -> PA5.

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

struct ProbeReg {
    uint8_t reg;
    uint8_t len;           // bytes to clock in after the command
    const char* name;
};

// Multi-byte reads: first byte is the dummy-cycle garbage, print it too.
constexpr ProbeReg probes[] = {
    {0x0A, 2, "RDDPM   "},  // power mode: expect ~0x9C after DISPON
    {0x0B, 2, "RDDMADCTL"},
    {0x0C, 2, "RDDCOLMOD"},
    {0x04, 4, "RDDID   "},
    {0x09, 5, "RDDST   "},
    {0xD3, 4, "ID4     "},  // ILI9341/9486/9488 answer .. 00 93/94 41/86/88
    {0xBF, 6, "DEVCODE "},  // ILI9481 device code read: .. 02 04 94 81 ..
};
constexpr uint8_t probe_count = sizeof(probes) / sizeof(probes[0]);

struct Prober : brio::Fsm<Prober, Tick, brio::SpiDone> {
    static inline brio::EventQueue<Event, 3, P> queue;
    static inline brio::TimeEvent<P, Prober, Tick> timer{Tick{}};

    static inline uint8_t phase = 0;
    static inline uint8_t idx = 0;
    static inline uint8_t cur_cmd = 0;
    static inline uint8_t buf[8];

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

    // SLPOUT, wait 150 ms, DISPON, wait 25 ms -> probing
    static Status waking(const Event& e) {
        return brio::match(e,
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
                return transition(&probing);
            },
            [](auto) { return unhandled(); }
        );
    }

    static Status probing(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                idx = 0;
                start_read();
                return handled();
            },
            [](brio::SpiDone d) {
                report(d);
                if (++idx < probe_count) {
                    start_read();
                } else {
                    brio::print(serial, "----", brio::crlf);
                    timer.arm(brio::ticks_from_ms<P>(1000));
                }
                return handled();
            },
            [](Tick) {
                idx = 0;
                start_read();
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }

private:
    static void send_cmd(uint8_t c) {
        cur_cmd = c;
        brio::post<Bus>(SpiHw::Request{
            CsPin::ref(), DcPin::ref(),
            brio::lend<brio::Lease::reply>(&cur_cmd), 1,
            {}, {}, 0,
            brio::reply_to<Prober, brio::SpiDone>()});
    }

    static void start_read() {
        cur_cmd = probes[idx].reg;
        for (uint8_t i = 0; i < sizeof(buf); ++i) {
            buf[i] = 0;
        }
        brio::post<Bus>(SpiHw::Request{
            CsPin::ref(), DcPin::ref(),
            brio::lend<brio::Lease::reply>(&cur_cmd), 1,
            {}, brio::lend<brio::Lease::reply>(buf), probes[idx].len,     // rx-only data phase
            brio::reply_to<Prober, brio::SpiDone>()});
    }

    static void report(brio::SpiDone d) {
        brio::print(serial, brio::hex(probes[idx].reg), " ",
                    probes[idx].name, " -> ");
        if (d.status != brio::spi_ok) {
            brio::print(serial, "status=", d.status, brio::crlf);
            return;
        }
        for (uint8_t i = 0; i < probes[idx].len; ++i) {
            brio::print(serial, brio::hex(buf[i]), " ");
        }
        brio::print(serial, brio::crlf);
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
                "display ID probe: needs module SDO wired to PA5",
                brio::crlf);

    brio::Kernel<P, Prober, Bus>::run();
}
