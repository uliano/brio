// i2c_scan - first on-silicon test of the I2C stack: an address scanner
// on TWI0 (SDA PA2, SCL PA3, external pull-ups), the I2C twin of
// spi_loopback.
//
// A Scanner sweeps 0x08..0x77 once every 2 s, one empty request (the
// address probe: S addr+W P) per address, sequentially through the
// I2cBus arbiter - the next probe is posted from the I2cDone handler of
// the previous one, so at most one request is ever in flight. Every
// ACK is printed as it comes; the sweep ends with a summary line:
//
//   I2C scan #3: 0x60               (one device: the MCP47CVB22 DAC)
//   I2C scan #4: no device answered (wiring? pull-ups?)
//
// Any status other than "ACK"/"no ACK" (arbitration lost, bus error)
// is printed with its code: it means the wire is misbehaving, not
// merely empty. Serial console @ 460800 on USART2 ALT1 (PF4/PF5).

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrdx/clock.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/twi.hpp"
#include "avrdx/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/print.hpp"

using P = brio::AvrPlatform;

// The clock: the ONE truth about CLK_PER for every driver of this
// target (avrdx/clock.hpp). 24 MHz crystal on PA0/PA1, OSCHF fallback at
// the same rate; `clock` is an empty tag passed to driver inits.
using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

using TwiHw = brio::Twi<0>;                        // PA2 SDA / PA3 SCL
using I2c = brio::I2cBus<TwiHw, P>;

struct Kick {};                                    // start a sweep

struct Scanner : brio::Fsm<Scanner, Kick, brio::I2cDone> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static inline brio::TimeEvent<P, Scanner, Kick> cadence{Kick{}};

    static constexpr uint8_t first_addr = 0x08;    // below: reserved
    static constexpr uint8_t last_addr = 0x77;     // above: 10-bit/reserved

    static inline uint16_t sweep = 0;
    static inline uint8_t addr = first_addr;
    static inline uint8_t found = 0;

    static void init() { start(&idle); }

    static Status idle(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                cadence.arm_every(brio::ticks_from_ms<P>(2000));
                return handled();
            },
            [](Kick) {
                ++sweep;
                addr = first_addr;
                found = 0;
                brio::print(serial, "I2C scan #", sweep, ":");
                return transition(&sweeping);
            },
            [](auto) { return unhandled(); }
        );
    }

    static Status sweeping(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                probe(addr);
                return handled();
            },
            [](brio::I2cDone d) {
                if (d.status == brio::i2c_ok) {
                    ++found;
                    brio::print(serial, " ", brio::hex(addr));
                } else if (d.status != brio::i2c_nack_addr) {
                    brio::print(serial, " [", brio::hex(addr),
                                " status=", d.status, "]");
                }
                if (addr == last_addr) {
                    if (found == 0) {
                        brio::print(serial, " no device answered",
                                    " (wiring? pull-ups?)");
                    }
                    brio::print(serial, brio::crlf);
                    return transition(&idle);
                }
                probe(++addr);
                return handled();
            },
            [](Kick) { return handled(); },   // a sweep is running: skip
            [](auto) { return unhandled(); }
        );
    }

private:
    static void probe(uint8_t a) {
        brio::post<I2c>(TwiHw::Request{
            a, nullptr, 0, nullptr, 0,             // empty: address only
            brio::reply_to<Scanner, brio::I2cDone>()});
    }
};

} // namespace

// ---- target glue ------------------------------------------------------------
ISR(TWI0_TWIM_vect) {
    if (TwiHw::isr()) {
        brio::post<I2c>(brio::TransferDone{TwiHw::status()});
    }
}
ISR(USART2_RXC_vect) { Serial::rxc(); }            // console is output-only here
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    SysClock::init();
    Serial::init(clock, 460800);
    TwiHw::init(clock);
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf,
                "I2C scanner: TWI0 on PA2(SDA)/PA3(SCL), 100 kHz",
                brio::crlf);

    brio::Kernel<P, Scanner, I2c>::run();
}
