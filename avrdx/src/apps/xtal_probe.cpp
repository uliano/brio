// xtal_probe - the crystal diagnosis probe, the bench tool a board
// whose 24 MHz crystal will not start gets flashed with (board B is
// the first patient). The main clock stays on the internal OSCHF for
// the whole run: the XOSCHF oscillator is only STARTED and OBSERVED
// (MCLKSTATUS.EXTS), never switched to, so the console keeps running
// whatever the crystal does.
//
// It sweeps the whole option space of the oscillator: every FRQRANGE
// (the range code also sets the driver's transconductance - the 32M
// range on a 24 MHz crystal is the "more drive" experiment that can
// separate a gain-margin problem, cured by smaller load capacitors,
// from a dead joint) x every CSUTHF start-up time. For each combo it
// reports stable-or-not and how many polling spins stability took -
// a healthy crystal comes up in a handful of spins at 4k CSUT, a
// marginal one shows up as a large or run-to-run wandering count.
//
// Console: USART2 ALT1 (PF4/PF5) 460800. 'r' repeats the sweep,
// anything else prints help.

// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 24'000'000>;
constexpr SysClock clock;

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

namespace {

using namespace brio;

/// One start attempt: enable with the range implied by `range_hz` and
/// the given start-up time, poll EXTS up to `budget` spins, stop.
/// Returns the spins consumed, or 0 on timeout (0 spins never happens
/// on success: even a warm oscillator needs the synchronizer).
uint32_t attempt(uint32_t range_hz, XoschfStartup sut, uint32_t budget) {
    Xoschf::start_crystal(range_hz, sut);
    uint32_t spins = 0;
    while (!Xoschf::stable() && spins < budget) {
        ++spins;
    }
    const bool ok = Xoschf::stable();
    Xoschf::stop();
    return ok ? (spins ? spins : 1) : 0;
}

void sweep() {
    struct Range { uint32_t hz; const char* name; };
    static constexpr Range ranges[] = {
        {8'000'000, " 8M"}, {16'000'000, "16M"}, {24'000'000, "24M"}, {32'000'000, "32M"},
    };
    static constexpr XoschfStartup suts[] = {
        XoschfStartup::cycles256, XoschfStartup::cycles1k, XoschfStartup::cycles4k,
    };
    static constexpr const char* sut_names[] = {"256 ", "1k  ", "4k  "};
    // ~0.5 s of polling at 24 MHz: orders of magnitude beyond any
    // crystal's start-up, so a timeout means "does not oscillate".
    constexpr uint32_t budget = 3'000'000;

    print(serial, "FRQRANGE x CSUTHF sweep (spins to EXTS, 0 = never):", crlf);
    for (const Range& r : ranges) {
        print(serial, "  ", r.name, ": ");
        for (uint8_t s = 0; s < 3; ++s) {
            const uint32_t spins = attempt(r.hz, suts[s], budget);
            print(serial, sut_names[s]);
            if (spins == 0) print(serial, "never");
            else print(serial, spins);
            print(serial, s < 2 ? "  " : "");
        }
        print(serial, crlf);
    }
    print(serial, "done ('r' repeats)", crlf);
}

} // namespace

int main() {
    SysClock::init();
    Serial::init(clock, 460800);
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "xtal_probe - XOSCHF crystal diagnosis (board ", board,
          ", main clock stays on OSCHF, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
    sweep();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == 'r' || c == 'R') sweep();
        else print(serial, "r = repeat the sweep", crlf);
        print(serial, "> ");
    }
}
