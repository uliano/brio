// wire_check - what is actually wired, measured instead of believed.
//
// A bench instrument, not a suite: it drives ONE pad at a time and
// reports the level of every other pad it knows, so the host can build
// the whole connection matrix - which catches a wire on the WRONG pin
// (it follows a pad nobody expected) as clearly as a missing one.
//
// The protocol on the console, one line each way:
//   n            -> "n <count> PA0 PA2 ..."   the pad names, in order
//   i            -> "ok"    every pad an input with its pull-up
//   d <ix> <lvl> -> "ok"    every pad an input, then pad <ix> driven
//   r            -> "r 1011..."  one character a pad, in the same order
//
// The reader always pulls UP and the driver always drives both levels,
// so a connected pad reads 0 then 1, an unconnected one 1 then 1, and a
// pad shorted to ground 0 then 0 - and an external pull-up on the wire
// (the I2C pair) changes none of that, because a driven low wins.
//
// The pads are this bench: the four SPI2 lines, the two I2C1
// lines, USART3's pair, the KEY pad the peer drives to wake this board,
// the crossed USART2/UART4 pair on the board itself, and the five pads
// the optional straps use. The console's own two (PA9/PA10), the LED and
// the two crystals are not here and are never touched.
//
// build: boards = v203c6,v203c8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v203/clock.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/usart.hpp"
#include "util/print.hpp"

using P = brio::Ch32v203Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

using Serial = brio::Uart<1, P>;
constexpr Serial serial;

namespace {

struct Pad {
    const char* name;
    void (*idle)();
    void (*drive)(bool);
    bool (*read)();
};

template <char port, uint8_t pin>
constexpr Pad pad(const char* name)
{
    using Pn = brio::Pin<port, pin>;
    return Pad{name,
               [] { Pn::input(brio::PinPull::up); },
               [](bool level) { Pn::output(level); },
               [] { return Pn::read(); }};
}

constexpr Pad pads[] = {
    pad<'A', 0>("PA0"), pad<'A', 1>("PA1"), pad<'A', 2>("PA2"), pad<'A', 3>("PA3"),
    pad<'A', 4>("PA4"), pad<'A', 5>("PA5"), pad<'A', 6>("PA6"), pad<'A', 7>("PA7"),
    pad<'A', 8>("PA8"), pad<'A', 11>("PA11"), pad<'A', 12>("PA12"), pad<'A', 15>("PA15"),
    pad<'B', 0>("PB0"), pad<'B', 1>("PB1"), pad<'B', 2>("PB2"), pad<'B', 3>("PB3"),
    pad<'B', 4>("PB4"), pad<'B', 5>("PB5"), pad<'B', 6>("PB6"), pad<'B', 7>("PB7"),
    pad<'B', 8>("PB8"), pad<'B', 9>("PB9"), pad<'B', 10>("PB10"), pad<'B', 11>("PB11"),
    pad<'B', 12>("PB12"), pad<'B', 13>("PB13"), pad<'B', 14>("PB14"), pad<'B', 15>("PB15"),
    pad<'C', 13>("PC13"), pad<'D', 0>("PD0"), pad<'D', 1>("PD1"),
};
constexpr uint8_t pad_count = sizeof(pads) / sizeof(pads[0]);

void all_idle()
{
    for (const Pad& p : pads) { p.idle(); }
}

/// The decimal number starting at *s, and s left on what follows it.
uint8_t number(const char*& s)
{
    while (*s == ' ') { ++s; }
    uint8_t v = 0;
    while (*s >= '0' && *s <= '9') { v = static_cast<uint8_t>(v * 10 + (*s - '0')); ++s; }
    return v;
}

void execute(const char* line)
{
    switch (line[0]) {
    case 'n': {
        brio::print(serial, "n ", pad_count);
        for (const Pad& p : pads) { brio::print(serial, " ", p.name); }
        brio::print(serial, brio::crlf);
        break;
    }
    case 'i':
        all_idle();
        brio::print(serial, "ok", brio::crlf);
        break;
    case 'd': {
        const char* s = line + 1;
        const uint8_t ix = number(s);
        const uint8_t level = number(s);
        all_idle();
        if (ix < pad_count) {
            pads[ix].drive(level != 0u);
            brio::print(serial, "ok", brio::crlf);
        } else {
            brio::print(serial, "err", brio::crlf);
        }
        break;
    }
    case 'r': {
        brio::print(serial, "r ");
        for (const Pad& p : pads) { brio::print(serial, p.read() ? "1" : "0"); }
        brio::print(serial, brio::crlf);
        break;
    }
    default:
        brio::print(serial, "err", brio::crlf);
        break;
    }
}

}  // namespace

int main()
{
    SysClock::init();
    Serial::init(clock, 115200);
    brio::enable_interrupts();
    all_idle();
    brio::print(serial, brio::crlf, "wire_check ", brio::device::part_name, brio::crlf);

    char line[24];
    uint8_t n = 0;
    for (;;) {
        uint8_t b = 0;
        if (!Serial::read_byte(b)) { continue; }
        const char c = static_cast<char>(b);
        if (c == '\r' || c == '\n') {
            if (n != 0u) {
                line[n] = '\0';
                execute(line);
                n = 0;
            }
            continue;
        }
        if (n < sizeof(line) - 1u) { line[n++] = c; }
    }
}

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { Serial::isr(); }
