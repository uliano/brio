// mcp_diag - MCP3550 behaviour probe, bit-banged, no kernel, driven one
// experiment at a time from the serial console so a logic analyzer on
// CS/SCK/SDO can capture exactly one known sequence per trigger.
//
// Why: the arbitrated SPI engine owns CS and asserts it only inside a
// request, so the ADC's classic "hold CS low and wait for RDY" idiom is
// not available as such - before writing the ADC client we need to know
// on the wire how this part behaves with CS toggled, with early clocks,
// and what its real t_conv is.
//
// Wiring: CS = PB0, SCK = PA6, SDO/RDY = PA5 (the SPI0 pins used as
// plain GPIO - SPI0 stays disabled). SPI mode 1,1: SCK idles high, the
// device shifts on the falling edge, we latch on the rising edge, MSB
// first. SCK is deliberately slow (10 us half period, 50 kHz) so a cheap
// analyzer resolves every edge. The DAC on I2C is not touched: VIN is
// whatever VOUT0 last was.
//
// Console @ 460800: press a key, get one experiment (each starts from a
// 600 ms CS-high rest so the part is in a known state):
//   a  classic: CS low, wait RDY low (t_conv), 24 clocks, CS high
//   b  CS toggle: CS low 1 ms, CS high 150 ms, CS low: RDY at once?
//      then wait RDY anyway, read
//   c  early clocks: CS low, 8 clocks at once, wait RDY, read
//   d  continuous: read, keep CS low, does RDY come again? read again
//   e  MISO level with CS high (another driver on the net?)
//   g  MOSI -> MISO short (leftover loopback jumper)?
//   h  double read right after RDY (second must be all ones)
//   f  RDY trace: one char per 5 ms, CS low, no clocks; read; trace
//   k  hold: convert to the end with CS low, CS high 200 ms, CS low:
//      is the result still there?
//   x  dac_adc's sequence: 8-clock trigger, CS high 120 ms, read
//   y  same with a bare 30 us CS pulse as trigger
//   z  the x sequence through the hardware Spi<0> engine (polled), mode 3
//   Z  same, mode 0
//   t  timing marker: CS 3 short pulses (find t=0 on the analyzer)
//   ?  this list
// Every experiment prints what it did and what it saw.
//
// Bench log 2026-08-17 (analyzer on CS/SCK/SDO), what this part does:
//  - t_conv 81 ms with CS held low; RDY low then valid frame (a, h);
//    a second read gives all ones (h) - single conversion mode.
//  - CS raised DURING a conversion: it goes on and the result is HELD
//    for the next CS fall (b, x, y) - but it then takes ~119 ms from
//    the trigger, not 81 (measured on the analyzer: RDY at 119.1 ms).
//  - conversion completed with CS LOW, not read, CS high, CS low: a NEW
//    conversion starts (k) - the result is held only if CS was high at
//    completion.
//  - clocks during the conversion are harmless (c, x).
//  - after CS falls the device needs a few us before SDO is valid; the
//    hardware engine clocking 1.5 us after CS lost the frame (0x7FFFFF)
//    - hence Spi::Request::cs_setup_us.
//  - the device latches its SPI mode from SCK at CS fall: the engine
//    have SCK at CPOL before asserting CS (Spi::apply_mode changes the
//    mode with the peripheral disabled), the AVR SPI does not move SCK
//    on a CTRLB write while enabled.
//  Earlier chaos (random t_conv, frames decaying to zero) was a PA5
//  header pin that was not soldered.
// Against the datasheet (DS20001950F): 5.3 confirms "CS rise during
// tCONV -> conversion completes, then Shutdown; the next CS fall does
// not restart"; 5.3.1: in single conversion mode RDY is LATCHED at each
// CS fall and does not update while CS stays low; 5.5: mode 1,1 needs
// SCK idling high and RDY tested before the first clock edge (mode 0,0
// = 25 clocks, RDY as first bit); tCSL = 8 us minimum CS low; the first
// conversion after Shutdown is longer by 144 fOSC periods (~5 ms). NOT
// in the datasheet: a CS-to-first-SCK setup in us (tRDY <= 50 ns; only
// "an internal power-up delay must be observed" when exiting Shutdown)
// and the ~119 ms trigger-to-RDY with CS high - both are bench facts.

#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/spi.hpp"
#include "avrdx/uart.hpp"
#include "util/print.hpp"

namespace {

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

using Cs = brio::Pin<'B', 0>;
using Sck = brio::Pin<'A', 6>;
using Sdo = brio::Pin<'A', 5>;
using Mosi = brio::Pin<'A', 4>;

constexpr uint16_t ready_timeout_ms = 600;

/// Wait for RDY (SDO low) with CS low; returns elapsed ms, or -1 on timeout.
int16_t wait_ready(uint16_t timeout_ms = ready_timeout_ms) {
    _delay_us(50);   // SDO needs a moment after CS falls before it shows RDY
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

/// n SCK pulses at 50 kHz, MSB first, latch on the rising edge.
uint32_t clock_bits(uint8_t n) {
    uint32_t raw = 0;
    for (uint8_t i = 0; i < n; ++i) {
        Sck::clear();
        _delay_us(10);
        Sck::set();
        raw = (raw << 1) | (Sdo::read() ? 1u : 0u);
        _delay_us(10);
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

void read_and_report(int16_t t) {
    brio::print(serial, "t=", t, " ms  ");
    if (t >= 0) {
        decode(clock_bits(24));
    } else {
        brio::print(serial, "(RDY never came)");
    }
}

void exp_a() {
    brio::print(serial, "a classic:   ");
    rest();
    Cs::clear();
    read_and_report(wait_ready());
    Cs::set();
    brio::print(serial, brio::crlf);
}

void exp_b() {
    brio::print(serial, "b CS toggle: ");
    rest();
    Cs::clear();                       // start a conversion
    _delay_ms(1);
    Cs::set();                         // ...and leave the bus
    _delay_ms(150);                    // longer than any t_conv
    Cs::clear();
    _delay_us(20);
    const bool ready_at_once = !Sdo::read();
    brio::print(serial, "RDY low at CS fall=", ready_at_once, "  then ");
    read_and_report(wait_ready());
    Cs::set();
    brio::print(serial, brio::crlf);
}

void exp_c() {
    brio::print(serial, "c early clk: ");
    rest();
    Cs::clear();
    const uint32_t early = clock_bits(8);   // 8 clocks while RDY is high
    brio::print(serial, "early=", brio::hex(early), "  then ");
    read_and_report(wait_ready());
    Cs::set();
    brio::print(serial, brio::crlf);
}

void exp_d() {
    brio::print(serial, "d contin.:   first ");
    rest();
    Cs::clear();
    read_and_report(wait_ready());
    _delay_us(20);
    const bool high_after_read = Sdo::read();
    brio::print(serial, "  RDY high after read=", high_after_read, "  second ");
    read_and_report(wait_ready());
    Cs::set();
    brio::print(serial, brio::crlf);
}

void exp_e() {
    Cs::set();
    uint8_t lows = 0;
    for (uint8_t i = 0; i < 100; ++i) {
        if (!Sdo::read()) ++lows;
        _delay_ms(1);
    }
    brio::print(serial, "e CS high:   MISO low ", lows, "/100 samples", brio::crlf);
}

void exp_g() {
    Cs::set();
    Mosi::clear(); Mosi::output(); _delay_us(10);
    const bool follows_low = !Sdo::read();
    Mosi::set(); _delay_us(10);
    const bool follows_high = Sdo::read();
    Mosi::input();
    brio::print(serial, "g MOSI->MISO: follows low=", follows_low,
                " high=", follows_high,
                (follows_low && follows_high) ? "  <- JUMPER PA4-PA5 PRESENT" : "",
                brio::crlf);
}

void exp_h() {
    brio::print(serial, "h double:    first ");
    rest();
    Cs::clear();
    read_and_report(wait_ready());
    brio::print(serial, "  second ");
    decode(clock_bits(24));
    Cs::set();
    brio::print(serial, brio::crlf);
}

void trace(uint8_t samples) {
    for (uint8_t i = 0; i < samples; ++i) {
        brio::print(serial, Sdo::read() ? '1' : '0');
        _delay_ms(5);
    }
}

void exp_f() {
    brio::print(serial, "f trace (5 ms/char, 1=busy 0=ready), CS low, no clocks:",
                brio::crlf, "   ");
    rest();
    Cs::clear();
    trace(60);                          // 300 ms
    brio::print(serial, brio::crlf, "   read: ");
    decode(clock_bits(24));
    brio::print(serial, brio::crlf, "   after read: ");
    trace(60);
    Cs::set();
    brio::print(serial, brio::crlf);
}

/// k. hold: convert to completion with CS LOW, no clocks, do NOT read;
/// CS high 200 ms; CS low again: is the result waiting (RDY low at once,
/// valid data)? This is the two-tenure scheme a shared bus could use.
void exp_k() {
    brio::print(serial, "k hold:      conv ");
    rest();
    Cs::clear();
    const int16_t t = wait_ready();
    brio::print(serial, "t=", t, " ms, CS high 200 ms, CS low: ");
    _delay_ms(10);
    Cs::set();
    _delay_ms(200);
    Cs::clear();
    _delay_us(50);
    const bool ready_at_once = !Sdo::read();
    brio::print(serial, "RDY low at once=", ready_at_once, "  ");
    read_and_report(wait_ready());
    Cs::set();
    brio::print(serial, brio::crlf);
}

/// x. exactly dac_adc's sequence: CS low, 8 clocks, CS high, 120 ms,
/// CS low, RDY?, 24 clocks. If bit-bang succeeds here the engine path
/// is at fault; if it fails, the clocked trigger is.
void exp_x() {
    brio::print(serial, "x dac_adc seq: ");
    rest();
    Cs::clear();
    const uint32_t early = clock_bits(8);
    Cs::set();
    _delay_ms(120);
    Cs::clear();
    _delay_us(50);
    const bool ready_at_once = !Sdo::read();
    brio::print(serial, "early=", brio::hex(early), " RDY low at once=", ready_at_once, "  ");
    read_and_report(wait_ready());
    Cs::set();
    brio::print(serial, brio::crlf);
}

/// y. same with a clock-free trigger: CS low 30 us, CS high, 120 ms, read.
void exp_y() {
    brio::print(serial, "y bare trig:   ");
    rest();
    Cs::clear();
    _delay_us(30);
    Cs::set();
    _delay_ms(120);
    Cs::clear();
    _delay_us(50);
    const bool ready_at_once = !Sdo::read();
    brio::print(serial, "RDY low at once=", ready_at_once, "  ");
    read_and_report(wait_ready());
    Cs::set();
    brio::print(serial, brio::crlf);
}

/// z. the same two-request sequence through the hardware Spi<0> engine
/// (polled, no kernel): mode 3, div64, 1-byte trigger, 120 ms, 3-byte
/// read. Isolates the engine path from the protocol.
void exp_z(uint8_t mode, uint16_t trig_len, brio::SpiClock clk, bool prime = false, bool manual_cs = false) {
    using SpiHw = brio::Spi<0>;
    brio::print(serial, "z engine trig ", trig_len, " B prime=", prime, " manualCS=", manual_cs, ": ");
    rest();
    SpiHw::init();
    static uint8_t rx[16];
    if (prime) {
        // one dummy byte with NO chip select: parks SCK at CPOL (high in
        // mode 3) so the next CS falling edge finds it there
        SpiHw::Request dummy{{}, {}, nullptr, 0, nullptr, rx, 1, {}, clk, mode, true};
        SpiHw::start(dummy);
        _delay_us(20);
    }
    const brio::PinRef cs = manual_cs ? brio::PinRef{} : Cs::ref();
    SpiHw::Request trig{cs, {}, nullptr, 0, nullptr, rx, trig_len, {},
                        clk, mode, true};
    SpiHw::Request read{cs, {}, nullptr, 0, nullptr, rx, 3, {},
                        clk, mode, true};
    if (manual_cs) { Cs::clear(); _delay_us(50); }
    SpiHw::start(trig);
    if (manual_cs) { Cs::set(); }
    const uint8_t early = rx[0];
    _delay_ms(120);
    if (manual_cs) { Cs::clear(); _delay_us(50); }
    SpiHw::start(read);
    if (manual_cs) { Cs::set(); }
    const uint32_t raw = (static_cast<uint32_t>(rx[0]) << 16) |
                         (static_cast<uint32_t>(rx[1]) << 8) | rx[2];
    SPI0.CTRLA = 0;                       // back to GPIO for the other experiments
    Sck::set(); Sck::output();
    Sdo::input();
    Mosi::input();
    Cs::set();
    brio::print(serial, "early=", brio::hex(early), "  ");
    decode(raw);
    brio::print(serial, brio::crlf);
}

/// i. the read through the ISR pump (as dac_adc does): SPI0 vector bound
/// below, non-polled request, wait for completion. Trigger polled.
volatile bool spi_done = false;
void exp_i(bool isr_trigger) {
    using SpiHw = brio::Spi<0>;
    brio::print(serial, "i ISR pump (trigger via ISR=", isr_trigger, "): ");
    rest();
    SpiHw::init();
    static uint8_t rx[3];
    SpiHw::Request trig{Cs::ref(), {}, nullptr, 0, nullptr, rx, 1, {},
                        brio::SpiClock::div64, SPI_MODE_3_gc, !isr_trigger};
    SpiHw::Request read{Cs::ref(), {}, nullptr, 0, nullptr, rx, 3, {},
                        brio::SpiClock::div64, SPI_MODE_3_gc, false};
    spi_done = false;
    if (!SpiHw::start(trig)) {
        while (!spi_done) {}
    }
    _delay_ms(120);
    spi_done = false;
    SpiHw::start(read);
    while (!spi_done) {}
    const uint32_t raw = (static_cast<uint32_t>(rx[0]) << 16) |
                         (static_cast<uint32_t>(rx[1]) << 8) | rx[2];
    SPI0.CTRLA = 0;
    Sck::set(); Sck::output();
    Sdo::input();
    Mosi::input();
    Cs::set();
    decode(raw);
    brio::print(serial, brio::crlf);
}

void exp_t() {
    brio::print(serial, "t marker: 3 x (CS low 100 us, high 100 us)", brio::crlf);
    Cs::set();
    for (uint8_t i = 0; i < 3; ++i) {
        Cs::clear(); _delay_us(100);
        Cs::set();   _delay_us(100);
    }
}

void help() {
    brio::print(serial,
        "MCP3550 diag - CS PB0, SCK PA6 (50 kHz), SDO/RDY PA5", brio::crlf,
        "  a classic  b CS toggle  c early clocks  d continuous", brio::crlf,
        "  e MISO idle  g jumper?  h double read  f RDY trace", brio::crlf,
        "  k hold (convert, CS high, read later)  t marker", brio::crlf,
        "  x dac_adc sequence (8-clock trigger)  y bare CS-pulse trigger", brio::crlf,
        "  z same through the Spi<0> engine, mode 3  Z ... mode 0", brio::crlf);
}

} // namespace

ISR(SPI0_INT_vect) { if (brio::Spi<0>::isr()) { spi_done = true; } }
ISR(USART2_RXC_vect) { Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

int main() {
    brio::init_clock_24mhz();
    Serial::init(460800);
    Cs::set();  Cs::output();
    Sck::set(); Sck::output();         // idles high (mode 1,1)
    Sdo::input();
    Mosi::input();
    // Deselect the other bus citizens (display PD0, touch PD5), quiet display.
    brio::Pin<'D', 0>::set(); brio::Pin<'D', 0>::output();
    brio::Pin<'D', 1>::set(); brio::Pin<'D', 1>::output();
    brio::Pin<'D', 2>::set(); brio::Pin<'D', 2>::output();
    brio::Pin<'D', 5>::set(); brio::Pin<'D', 5>::output();
    sei();

    brio::print(serial, brio::crlf);
    help();
    for (;;) {
        uint8_t key;
        if (!Serial::read_byte(key)) {
            continue;
        }
        switch (key) {
            case 'a': exp_a(); break;
            case 'b': exp_b(); break;
            case 'c': exp_c(); break;
            case 'd': exp_d(); break;
            case 'e': exp_e(); break;
            case 'g': exp_g(); break;
            case 'h': exp_h(); break;
            case 'f': exp_f(); break;
            case 'k': exp_k(); break;
            case 'x': exp_x(); break;
            case 'y': exp_y(); break;
            case 'z': exp_z(SPI_MODE_3_gc, 1, brio::SpiClock::div64); break;
            case 'i': exp_i(false); break;
            case 'I': exp_i(true); break;
            case 'u': exp_z(SPI_MODE_3_gc, 1, brio::SpiClock::div64, true, false); break;
            case 'U': exp_z(SPI_MODE_3_gc, 1, brio::SpiClock::div64, true, true); break;
            case 'v': exp_z(SPI_MODE_3_gc, 1, brio::SpiClock::div64, false, true); break;
            case 'Z': exp_z(SPI_MODE_3_gc, 8, brio::SpiClock::div128); break;
            case 'w': exp_z(SPI_MODE_3_gc, 16, brio::SpiClock::div128); break;
            case 't': exp_t(); break;
            case '\r': case '\n': break;
            default: help(); break;
        }
    }
}
