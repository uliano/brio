// spi_loopback - first on-silicon test of the SPI stack: jumper PA4
// (MOSI) -> PA5 (MISO), and every byte transmitted comes straight back.
//
// A Tester fires a full-duplex transaction once per second (rolling
// 8-byte pattern, CS on PA7 - unconnected, just exercised), gets its
// SpiDone through the ReplyTo capsule and prints the verdict on the
// serial console:
//
//   SPI loopback #3: OK  (8 bytes)
//   SPI loopback #4: FAIL got FF FF ... (jumper missing?)
//
// Without the jumper MISO floats/reads 0xFF: FAIL is the expected
// no-jumper outcome, OK proves the whole chain - request event ->
// arbiter -> engine -> per-byte ISR -> TransferDone -> reply.

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

using SpiHw = brio::SpiHost<0>;                       // PA4/PA5/PA6
using Bus = brio::SpiBus<SpiHw, P>;
using CsPin = brio::Pin<'A', 7>;                  // exercised, not wired

struct Kick {};                                    // Tester heartbeat

struct Tester : brio::Fsm<Tester, Kick, brio::SpiDone> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static inline brio::TimeEvent<P, Tester, Kick> cadence{Kick{}};

    static constexpr uint8_t pattern_len = 8;
    static inline uint8_t tx[pattern_len];
    static inline uint8_t rx[pattern_len];
    static inline uint16_t round = 0;

    static void init() {
        CsPin::set();                              // idle high (inactive)
        CsPin::output();
        // Shared bus hygiene: DESELECT every real device on the bench
        // (display CS=PD0, ADC CS=PD3) and keep the display quiet
        // (RST=PD2 high, DC=PD1 high) while the jumper test runs.
        brio::Pin<'D', 0>::set(); brio::Pin<'D', 0>::output();
        brio::Pin<'D', 1>::set(); brio::Pin<'D', 1>::output();
        brio::Pin<'D', 2>::set(); brio::Pin<'D', 2>::output();
        brio::Pin<'D', 3>::set(); brio::Pin<'D', 3>::output();
        start(&running);
    }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                cadence.arm_every(brio::ticks_from_ms<P>(1000));
                return handled();
            },
            [](Kick) {
                ++round;
                for (uint8_t i = 0; i < pattern_len; ++i) {
                    tx[i] = static_cast<uint8_t>(round + i * 17);
                    rx[i] = 0;
                }
                brio::post<Bus>(SpiHw::Request{
                    CsPin::ref(), {},              // cs, no dc
                    {}, 0,                         // no command phase
                    brio::lend<brio::Lease::reply>(tx), brio::lend<brio::Lease::reply>(rx), pattern_len,  // full duplex
                    brio::reply_to<Tester, brio::SpiDone>()});
                return handled();
            },
            [](brio::SpiDone d) {
                report(d);
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }

private:
    static void report(brio::SpiDone d) {
        brio::print(serial, "SPI loopback #", round, ": ");
        if (d.status != brio::spi_ok) {
            brio::print(serial, "status=", d.status, brio::crlf);
            return;
        }
        bool ok = true;
        for (uint8_t i = 0; i < pattern_len; ++i) {
            if (rx[i] != tx[i]) {
                ok = false;
            }
        }
        if (ok) {
            brio::print(serial, "OK  (", pattern_len, " bytes)", brio::crlf);
        } else {
            brio::print(serial, "FAIL got ");
            for (uint8_t i = 0; i < pattern_len; ++i) {
                brio::print(serial, brio::hex(rx[i]), " ");
            }
            brio::print(serial, "(jumper PA4->PA5 missing?)", brio::crlf);
        }
    }
};

} // namespace

// ---- target glue ------------------------------------------------------------
ISR(SPI0_INT_vect) {
    if (SpiHw::isr()) {
        brio::post<Bus>(brio::TransferDone{brio::spi_ok});
    }
}
ISR(USART2_RXC_vect) { Serial::rxc(); }            // console is output-only here
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    SysClock::init();
    Serial::init(clock, 460800);
    SpiHw::init(clock);                                 // clock/mode travel per-request
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf,
                "SPI loopback tester: jumper PA4(MOSI) -> PA5(MISO)",
                brio::crlf);

    brio::Kernel<P, Tester, Bus>::run();
}
