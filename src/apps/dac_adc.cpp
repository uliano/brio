// dac_adc - two buses, one signal: the MCP47CVB22 DAC (I2C, TWI0 on
// PA2/PA3, addr 0x60) drives its VOUT0 into the MCP3550 22-bit ADC
// (SPI0, CS on PB0, SDO/RDY on MISO PA5). A ramp goes out on the DAC,
// each step is read back over I2C (write-then-read: the register-access
// idiom) and measured by the ADC, and both codes land on the serial
// console once per second:
//
//   step 4: DAC 0x800 (readback 0x800) -> ADC 1048320 (0x0FFF00) ovh=0 ovl=0
//
// With VDD as the reference on both sides the ADC code should track
// dac_code * 512 (12 bits -> 21 bits, VIN = VOUT0), less the DAC's and
// ADC's offset/gain error - a few LSB of the DAC. Anything else says
// which wire (or which reference assumption) is wrong.
//
// MCP3550 protocol through the arbitrated engine (CS owned by the
// engine, asserted only inside a request): the ADC's single-conversion
// mode is driven with TWO requests per sample -
//   1. trigger: a 1-byte read - the CS falling edge starts a conversion
//      (the byte clocked out meanwhile is just RDY high, discarded);
//   2. read, 200 ms later: a 3-byte read - RDY is already low when CS
//      falls, the 24-bit frame comes out [OVL][OVH][sign + 22-bit two's
//      complement], SPI mode 1,1. If the frame is all ones the device
//      was still busy (RDY high): the read is retried 20 ms later.
// This relies on the device keeping its conversion running while CS is
// high (single conversion mode with CS toggling), which is what makes
// the ADC a good citizen of a shared bus - the display can use SCK/MOSI
// while it converts. Measured on the bench (2026-08-17, analyzer): with
// CS held low the conversion ends at 83 ms; with CS HIGH during the
// conversion RDY comes ~119 ms after the trigger - the part converts
// slower in shutdown - and a read at 120 ms was on the knife edge. Also
// seen: after CS falls the ADC needs a few us before SDO shows RDY, and
// devices latching their SPI mode from SCK at CS-fall need SCK parked
// at CPOL beforehand (now guaranteed by the engine, Spi::park_sck).
//
// One BusDone alternative serves both buses: the FSM state says which
// bus the reply came from (a client of two buses distinguishes replies
// by its own state - the aliases SpiDone/I2cDone are the same type).

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrdx/clock.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/spi.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/twi.hpp"
#include "avrdx/uart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/wire.hpp"

using P = brio::AvrPlatform;

namespace {

template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

using TwiHw = brio::Twi<0>;                        // PA2 SDA / PA3 SCL
using I2c = brio::I2cBus<TwiHw, P>;
using SpiHw = brio::Spi<0>;                        // PA4/PA5/PA6
using Spi = brio::SpiBus<SpiHw, P>;

using AdcCs = brio::Pin<'B', 0>;                   // MCP3550 CS (moved from PD3, 2026-08-17)
constexpr uint8_t adc_cs_setup_us = 10;            // CS-to-first-SCK after wake (see spi.hpp)

// ---- MCP47CVB22 (dual 12-bit DAC, I2C) ---------------------------------------
// Command byte: [reg addr 7:3][cmd 2:1][x]; cmd 00 = write, 11 = read.
// Data: 16 bits MSB first, the 12-bit code right-aligned. Volatile
// registers: 0x00 DAC0, 0x01 DAC1, 0x08 VREF, 0x09 power-down, 0x0A gain.
constexpr uint8_t dac_addr = 0x60;                 // A0 to GND
constexpr uint8_t dac_reg_out0 = 0x00;
constexpr uint8_t dac_cmd_write = 0x00;
constexpr uint8_t dac_cmd_read = 0x06;

// ---- the sequence ------------------------------------------------------------
struct Kick {};                                    // once per second: next step
struct Tick {};                                    // settle / conversion timer

struct Loop : brio::Fsm<Loop, Kick, Tick, brio::BusDone> {
    static inline brio::EventQueue<Event, 4, P> queue;
    static inline brio::TimeEvent<P, Loop, Kick> cadence{Kick{}};
    static inline brio::TimeEvent<P, Loop, Tick> timer{Tick{}};

    static inline uint8_t step = 0;
    static inline uint16_t dac_code = 0;
    static inline uint8_t i2c_tx[3];
    static inline uint8_t i2c_rx[2];
    static inline uint8_t spi_rx[3];

    static void init() {
        AdcCs::set();
        AdcCs::output();
        // Shared bus hygiene: deselect the display (PD0) and touch (PD5),
        // keep the display quiet (RST PD2 high, DC PD1 high).
        brio::Pin<'D', 0>::set(); brio::Pin<'D', 0>::output();
        brio::Pin<'D', 1>::set(); brio::Pin<'D', 1>::output();
        brio::Pin<'D', 2>::set(); brio::Pin<'D', 2>::output();
        brio::Pin<'D', 5>::set(); brio::Pin<'D', 5>::output();
        start(&idle);
    }

    // idle: waiting for the next step of the ramp
    static Status idle(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                if (step == 0) {
                    cadence.arm_every(brio::ticks_from_ms<P>(1000));
                }
                return handled();
            },
            [](Kick) {
                dac_code = static_cast<uint16_t>(step * 512u);
                if (dac_code > 4095) {
                    dac_code = 4095;
                }
                return transition(&dac_write);
            },
            [](auto) { return unhandled(); },
        }, e);
    }

    // dac_write: I2C write of DAC0
    static Status dac_write(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                i2c_tx[0] = static_cast<uint8_t>((dac_reg_out0 << 3) | dac_cmd_write);
                brio::store_be16(&i2c_tx[1], dac_code);
                brio::post<I2c>(TwiHw::Request{
                    dac_addr, i2c_tx, 3, nullptr, 0,
                    brio::reply_to<Loop, brio::BusDone>(),
                    brio::I2cSpeed::fast_400k});
                return handled();
            },
            [](brio::BusDone d) {
                if (d.status != brio::i2c_ok) {
                    return fail("DAC write", d.status);
                }
                return transition(&dac_verify);
            },
            [](Kick) { return handled(); },        // still busy: skip a beat
            [](auto) { return unhandled(); },
        }, e);
    }

    // dac_verify: I2C write-then-read of DAC0 (repeated START)
    static Status dac_verify(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                i2c_tx[0] = static_cast<uint8_t>((dac_reg_out0 << 3) | dac_cmd_read);
                brio::post<I2c>(TwiHw::Request{
                    dac_addr, i2c_tx, 1, i2c_rx, 2,
                    brio::reply_to<Loop, brio::BusDone>(),
                    brio::I2cSpeed::fast_400k});
                return handled();
            },
            [](brio::BusDone d) {
                if (d.status != brio::i2c_ok) {
                    return fail("DAC readback", d.status);
                }
                return transition(&settling);
            },
            [](Kick) { return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }

    // settling: let VOUT0 and the ADC input settle before triggering
    static Status settling(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                timer.arm(brio::ticks_from_ms<P>(20));
                return handled();
            },
            [](Tick) { return transition(&triggering); },
            [](Kick) { return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }

    // triggering: CS falling edge starts a conversion (1 dummy byte)
    static Status triggering(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                brio::post<Spi>(SpiHw::Request{
                    AdcCs::ref(), {}, nullptr, 0,
                    nullptr, spi_rx, 1,
                    brio::reply_to<Loop, brio::BusDone>(),
                    brio::SpiClock::div64, SPI_MODE_3_gc});
                return handled();
            },
            [](brio::BusDone d) {
                if (d.status != brio::spi_ok) {
                    return fail("ADC trigger", d.status);
                }
                return transition(&converting);
            },
            [](Kick) { return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }

    // converting: with CS high the MCP3550 takes ~119 ms (measured), so
    // 200 ms leaves margin; a not-ready frame is retried anyway
    static Status converting(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                timer.arm(brio::ticks_from_ms<P>(200));
                return handled();
            },
            [](Tick) { return transition(&reading); },
            [](Kick) { return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }

    // reading: the 24-bit frame
    static Status reading(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                brio::post<Spi>(SpiHw::Request{
                    AdcCs::ref(), {}, nullptr, 0,
                    nullptr, spi_rx, 3,
                    brio::reply_to<Loop, brio::BusDone>(),
                    brio::SpiClock::div64, SPI_MODE_3_gc, false,
                    adc_cs_setup_us});
                return handled();
            },
            [](brio::BusDone d) {
                if (d.status != brio::spi_ok) {
                    return fail("ADC read", d.status);
                }
                if (brio::load_be24(spi_rx) == 0xFFFFFFul && retries < 10) {
                    ++retries;                     // RDY was still high
                    return transition(&retrying);
                }
                report();
                retries = 0;
                next_step();
                return transition(&idle);
            },
            [](Kick) { return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }

    // retrying: the frame said "busy" - wait a little and read again
    static Status retrying(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) {
                timer.arm(brio::ticks_from_ms<P>(20));
                return handled();
            },
            [](Tick) { return transition(&reading); },
            [](Kick) { return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }

    static inline uint8_t retries = 0;

private:
    static void report() {
        const uint32_t raw = brio::load_be24(spi_rx);
        const bool ovl = (raw & (1ul << 23)) != 0;
        const bool ovh = (raw & (1ul << 22)) != 0;
        const int32_t code =
            static_cast<int32_t>((raw & 0x3FFFFFul) << 10) >> 10;   // sign at bit 21
        const uint16_t readback = brio::load_be16(i2c_rx) & 0x0FFF;

        brio::print(serial, "step ", step, ": DAC ", brio::hex(dac_code),
                    (retries != 0) ? " (retried)" : "",
                    " (readback ", brio::hex(readback), ")",
                    " -> ADC ", code, " (", brio::hex(raw), ")",
                    " ovh=", ovh, " ovl=", ovl,
                    "  ratio*512=", (dac_code != 0) ? code / dac_code : 0,
                    brio::crlf);
    }

    static void next_step() {
        step = static_cast<uint8_t>((step + 1) % 9);   // 0, 512, ..., 4095
    }

    static Status fail(const char* what, uint8_t status) {
        brio::print(serial, "step ", step, ": ", what, " failed, status=",
                    status, brio::crlf);
        next_step();
        return transition(&idle);
    }
};

} // namespace

// ---- target glue ------------------------------------------------------------
ISR(TWI0_TWIM_vect) {
    if (TwiHw::isr()) {
        brio::post<I2c>(brio::TransferDone{TwiHw::status()});
    }
}
ISR(SPI0_INT_vect) {
    if (SpiHw::isr()) {
        brio::post<Spi>(brio::TransferDone{brio::spi_ok});
    }
}
ISR(USART2_RXC_vect) { Serial::rxc(); }            // console is output-only here
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    brio::init_clock_24mhz();
    Serial::init(460800);
    TwiHw::init();
    SpiHw::init();
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf,
                "DAC -> ADC loop: MCP47CVB22 VOUT0 (I2C 0x60) into MCP3550 (SPI, CS PB0)",
                brio::crlf);

    brio::Kernel<P, Loop, I2c, Spi>::run();
}
