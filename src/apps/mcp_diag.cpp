// mcp_diag - MCP3550 behaviour probe, bit-banged, no kernel: answers the
// questions the arbitrated SPI engine needs answered before it can talk
// to this ADC (CS is owned by the engine and asserted only inside a
// request, so the ADC's "hold CS low and wait for RDY" idiom is not
// available as such).
//
// Wiring as on the bench: CS = PD3, SCK = PA6, SDO/RDY = PA5 (the SPI0
// pins, used here as plain GPIO - SPI0 stays disabled). The DAC on I2C
// is not touched; VIN is whatever VOUT0 last was.
//
// Experiments, each from a 600 ms CS-high rest, results on the console:
//  A. classic: CS low, wait for RDY low (t_conv), clock 24 bits, CS high
//     -> proves the chip and gives t_conv;
//  B. CS toggle: CS low for 1 ms (starts a conversion), CS HIGH for
//     150 ms, CS low again: is RDY low at once (result held through the
//     CS-high period) or high (conversion aborted / restarted)? Then
//     wait for RDY anyway and read, to see what comes out;
//  C. clocks during conversion: CS low, 8 SCK pulses immediately (RDY
//     still high), keep CS low, wait for RDY, read 24 bits: does the
//     early clocking corrupt the frame?
//  D. continuous: read, keep CS low, does RDY come again after t_conv?
//  E. MISO level with the ADC deselected (another driver on the net?);
//  G. is MISO tied to MOSI (leftover loopback jumper)?
//  H. two reads back to back after RDY (second must be all ones);
//  F. RDY trace, one char per 5 ms, CS low, no clocks; then read; then
//     trace again - the poor man's logic analyzer.
// SPI mode 1,1: SCK idles high, the device shifts on the falling edge,
// we latch on the rising edge, MSB first.
//
// Bench log 2026-08-17: the chip answers and the value tracks the DAC
// (0x400dX for DAC 0x200: 262144 expected, +0.08%), but t_conv is
// random (0..290 ms for a fixed-80-ms part), data read a while after
// RDY comes back all zeros (F) while an immediate read is valid (H),
// and RDY sometimes is low at CS fall with empty data. Software cannot
// produce those: they read as spurious edges on SCK (data shifted out
// unasked) and on CS (conversions restarted) - wiring/decoupling on
// the ADC side, to be checked before dac_adc's ADC path is judged.

#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/uart.hpp"
#include "util/print.hpp"

namespace {

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

using Cs = brio::Pin<'D', 3>;
using Sck = brio::Pin<'A', 6>;
using Sdo = brio::Pin<'A', 5>;

/// Wait for RDY (SDO low) with CS low; returns elapsed ms, or -1 on timeout.
int16_t wait_ready(uint16_t timeout_ms) {
    for (uint16_t ms = 0; ms < timeout_ms; ++ms) {
        for (uint8_t i = 0; i < 10; ++i) {
            if (!Sdo::read()) {
                return static_cast<int16_t>(ms);
            }
            _delay_us(100);
        }
    }
    return -1;
}

uint32_t clock_bits(uint8_t n) {
    uint32_t raw = 0;
    for (uint8_t i = 0; i < n; ++i) {
        Sck::clear();
        _delay_us(2);
        Sck::set();
        raw = (raw << 1) | (Sdo::read() ? 1u : 0u);
        _delay_us(2);
    }
    return raw;
}

void decode(uint32_t raw) {
    const bool ovl = (raw & (1ul << 23)) != 0;
    const bool ovh = (raw & (1ul << 22)) != 0;
    const int32_t code = static_cast<int32_t>((raw & 0x3FFFFFul) << 10) >> 10;
    brio::print(serial, brio::hex(raw), " code=", code, " ovh=", ovh, " ovl=", ovl);
}

/// CS high long enough for any conversion in flight to end with CS high
/// (-> shutdown): every experiment starts from the same known state.
void rest() {
    Cs::set();
    _delay_ms(600);
}

void experiment_a() {
    rest();
    Cs::clear();
    const int16_t t = wait_ready(600);
    brio::print(serial, "A classic:  t_conv=", t, " ms  ");
    if (t >= 0) {
        decode(clock_bits(24));
    }
    Cs::set();
    brio::print(serial, brio::crlf);
}

void experiment_b() {
    rest();
    Cs::clear();                       // start a conversion
    _delay_ms(1);
    Cs::set();                         // ...and leave the bus
    _delay_ms(150);                    // longer than any t_conv
    Cs::clear();
    _delay_us(20);
    const bool ready_at_once = !Sdo::read();
    const int16_t t = wait_ready(600);
    brio::print(serial, "B toggle:   RDY low at CS fall=", ready_at_once,
                "  then t=", t, " ms  ");
    if (t >= 0) {
        decode(clock_bits(24));
    }
    Cs::set();
    brio::print(serial, brio::crlf);
}

void experiment_c() {
    rest();
    Cs::clear();
    const uint32_t early = clock_bits(8);   // 8 clocks while RDY is high
    const int16_t t = wait_ready(600);
    brio::print(serial, "C early clk: early=", brio::hex(early),
                " t_conv=", t, " ms  ");
    if (t >= 0) {
        decode(clock_bits(24));
    }
    Cs::set();
    brio::print(serial, brio::crlf);
}

/// D. continuous: after a read, keep CS low - does RDY rise at once and
/// fall again after t_conv (free-running conversions), and what does a
/// second read give?
void experiment_d() {
    rest();
    Cs::clear();
    const int16_t t1 = wait_ready(600);
    const uint32_t r1 = (t1 >= 0) ? clock_bits(24) : 0;
    _delay_us(20);
    const bool high_after_read = Sdo::read();
    const int16_t t2 = wait_ready(600);
    const uint32_t r2 = (t2 >= 0) ? clock_bits(24) : 0;
    Cs::set();
    brio::print(serial, "D contin.:  t1=", t1, " ms ");
    decode(r1);
    brio::print(serial, "  RDY high after read=", high_after_read,
                "  t2=", t2, " ms ");
    decode(r2);
    brio::print(serial, brio::crlf);
}

/// E. what the MISO net does with the ADC deselected (CS high): a level
/// count over 100 ms - anything but "floating/high" means another driver.
void experiment_e() {
    Cs::set();
    uint8_t lows = 0;
    for (uint8_t i = 0; i < 100; ++i) {
        if (!Sdo::read()) ++lows;
        _delay_ms(1);
    }
    brio::print(serial, "E CS high:  MISO low ", lows, "/100 samples", brio::crlf);
}

/// G. is MISO tied to MOSI (leftover loopback jumper)? Drive PA4 both
/// ways with the ADC deselected and see whether PA5 follows.
void experiment_g() {
    using Mosi = brio::Pin<'A', 4>;
    Cs::set();
    Mosi::clear(); Mosi::output(); _delay_us(10);
    const bool follows_low = !Sdo::read();
    Mosi::set(); _delay_us(10);
    const bool follows_high = Sdo::read();
    Mosi::input();
    brio::print(serial, "G MOSI->MISO: follows low=", follows_low,
                " high=", follows_high,
                (follows_low && follows_high) ? "  <- JUMPER PA4-PA5 PRESENT" : "",
                brio::crlf);
}

/// H. two reads back to back right after RDY: second one all zeros?
void experiment_h() {
    rest();
    Cs::clear();
    const int16_t t = wait_ready(600);
    brio::print(serial, "H double read: t=", t, " ms  first ");
    decode(clock_bits(24));
    brio::print(serial, "  second ");
    decode(clock_bits(24));
    Cs::set();
    brio::print(serial, brio::crlf);
}

/// F. RDY trace: one char per 5 ms (1 = high/busy, 0 = low/ready), CS
/// low throughout, no clocks; then a read and a second trace.
void trace(uint8_t samples) {
    for (uint8_t i = 0; i < samples; ++i) {
        brio::print(serial, Sdo::read() ? '1' : '0');
        _delay_ms(5);
    }
}
void experiment_f() {
    rest();
    Cs::clear();
    brio::print(serial, "F trace CS low, no clocks (5 ms/char):", brio::crlf, "   ");
    trace(60);                          // 300 ms
    brio::print(serial, brio::crlf, "   read: ");
    decode(clock_bits(24));
    brio::print(serial, brio::crlf, "   after read: ");
    trace(60);
    Cs::set();
    brio::print(serial, brio::crlf);
}

} // namespace

ISR(USART2_RXC_vect) { Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

int main() {
    brio::init_clock_24mhz();
    Serial::init(460800);
    Cs::set();  Cs::output();
    Sck::set(); Sck::output();         // idles high (mode 1,1)
    Sdo::input();
    // Deselect the other bus citizens (display PD0, touch PD5), quiet display.
    brio::Pin<'D', 0>::set(); brio::Pin<'D', 0>::output();
    brio::Pin<'D', 1>::set(); brio::Pin<'D', 1>::output();
    brio::Pin<'D', 2>::set(); brio::Pin<'D', 2>::output();
    brio::Pin<'D', 5>::set(); brio::Pin<'D', 5>::output();
    sei();

    brio::print(serial, brio::crlf, "MCP3550 diag (bit-bang PD3/PA6/PA5)",
                brio::crlf);
    for (;;) {
        experiment_a();
        experiment_b();
        experiment_c();
        experiment_d();
        experiment_e();
        experiment_g();
        experiment_h();
        experiment_f();
        brio::print(serial, brio::crlf);
        _delay_ms(1000);
    }
}
