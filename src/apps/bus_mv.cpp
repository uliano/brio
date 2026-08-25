// bus_mv - a disposable bench voltmeter for the I2C-bus investigation:
// board A measures, with its own ADC, everything the desk has no meter
// for. VDD and VDDIO2 come free through the internal dividers; the two
// bus nodes need one jumper each onto PORTD's analog pins:
//
//     A.PD1 (AIN1) -> the SDA node      A.PD2 (AIN2) -> the SCL node
//
// Each sweep reads VDD/10 and VDDIO2/10 against the 2.048 V internal
// reference (so the rails are absolute), then re-references the ADC to
// VDD and reads AIN1/AIN2 (so the nodes are reported both in mV and as
// a fraction of the rail the pins actually compare against). A sweep
// runs twice a second, forever: poke the wires and watch.
//
// Console: USART2 ALT1 (PF4/PF5) 460800. Any key prints nothing new -
// the loop is the interface.

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/adc.hpp"
#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/analog.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

namespace {

using namespace brio;

using Meter = Adc<0>;

constexpr AdcConfig against_2048{.reference = Ref::v2048, .accumulate = 16};
constexpr AdcConfig against_vdd{.reference = Ref::vdd, .accumulate = 16};

/// One settled reading of whatever is selected: a throw-away conversion
/// first (errata 2.3.2 hygiene after a mux change), then the kept one.
uint16_t settled() {
    (void)Meter::read();
    return Meter::read();
}

} // namespace

int main() {
    SysClock::init();
    Serial::init(clock, 460800);
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "bus_mv - bench voltmeter (board ", board,
          ", PD1 = SDA node, PD2 = SCL node)", crlf);

    for (;;) {
        // The rails, absolute: dividers against the 2.048 V reference.
        Meter::reconfigure(clock, against_2048);
        Meter::select(AdcInput::vdd_div10);
        const uint32_t vdd_mv = 10u * adc_mv(settled(), Meter::result_steps(), 2048);
        Meter::select(AdcInput::vddio2_div10);
        const uint32_t vddio2_mv = 10u * adc_mv(settled(), Meter::result_steps(), 2048);

        // The bus nodes, against the rail the digital pins compare with.
        Meter::reconfigure(clock, against_vdd);
        Meter::select(AnalogIn<Pin<'D', 1>>{});
        const uint32_t sda_counts = settled();
        Meter::select(AnalogIn<Pin<'D', 2>>{});
        const uint32_t scl_counts = settled();
        const uint32_t steps = Meter::result_steps();
        const uint32_t sda_mv = sda_counts * vdd_mv / steps;
        const uint32_t scl_mv = scl_counts * vdd_mv / steps;

        // The node truth A always has: its own digital taps. If PA2 reads
        // 1 while PD1's mV are low, the PD1 jumper is not on the node.
        print(serial, "VDD ", vdd_mv, " mV | VDDIO2 ", vddio2_mv,
              " mV | SDA ", sda_mv, " mV (", sda_counts * 100 / steps,
              "% of VDD) | SCL ", scl_mv, " mV (", scl_counts * 100 / steps,
              "%) | taps PA2=", Pin<'A', 2>::read() ? 1 : 0,
              " PA3=", Pin<'A', 3>::read() ? 1 : 0,
              " PC2=", Pin<'C', 2>::read() ? 1 : 0,
              " PC3=", Pin<'C', 3>::read() ? 1 : 0, crlf);
        delay_us(clock, 500'000);
    }
}
