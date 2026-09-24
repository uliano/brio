// wire_check - the peer half of the connection matrix.
//
// The same instrument and the same four-line protocol as the CH32V203's
// wire_check (ch32vx03/src/apps/wire_check.cpp): one pad driven at a
// time, every pad reported, the reader always pulling up. Built for the
// Nucleo-F446RE alone, because the pads below are that board's bench:
// the four SPI2 lines, I2C1's pair, USART1's pair and the one pad that
// drives a wake edge into the other board. The console is USART2 on
// PA2/PA3, the ST-LINK's virtual port, and is never touched here.
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"

using P = brio::Stm32f4Platform<>;
using SysClock =
    brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock;

constexpr brio::UartPins console_pins{
    .tx = {'A', 2, brio::PinFunction::af7},
    .rx = {'A', 3, brio::PinFunction::af7},
};
using Serial = brio::Uart<2, console_pins>;
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
    pad<'A', 0>("PA0"), pad<'A', 1>("PA1"), pad<'A', 4>("PA4"), pad<'A', 5>("PA5"),
    pad<'A', 6>("PA6"), pad<'A', 7>("PA7"), pad<'A', 8>("PA8"), pad<'A', 9>("PA9"),
    pad<'A', 10>("PA10"), pad<'A', 11>("PA11"), pad<'A', 12>("PA12"), pad<'A', 15>("PA15"),
    pad<'B', 0>("PB0"), pad<'B', 1>("PB1"), pad<'B', 2>("PB2"), pad<'B', 3>("PB3"),
    pad<'B', 4>("PB4"), pad<'B', 5>("PB5"), pad<'B', 6>("PB6"), pad<'B', 7>("PB7"),
    pad<'B', 8>("PB8"), pad<'B', 9>("PB9"), pad<'B', 10>("PB10"), pad<'B', 11>("PB11"),
    pad<'B', 12>("PB12"), pad<'B', 13>("PB13"), pad<'B', 14>("PB14"), pad<'B', 15>("PB15"),
    pad<'C', 0>("PC0"), pad<'C', 1>("PC1"), pad<'C', 2>("PC2"), pad<'C', 3>("PC3"),
    pad<'C', 4>("PC4"), pad<'C', 5>("PC5"), pad<'C', 6>("PC6"), pad<'C', 7>("PC7"),
    pad<'C', 8>("PC8"), pad<'C', 9>("PC9"), pad<'C', 10>("PC10"), pad<'C', 11>("PC11"),
    pad<'C', 12>("PC12"), pad<'C', 13>("PC13"), pad<'C', 14>("PC14"), pad<'C', 15>("PC15"),
    pad<'D', 2>("PD2"),
};
constexpr uint8_t pad_count = sizeof(pads) / sizeof(pads[0]);

void all_idle()
{
    for (const Pad& p : pads) { p.idle(); }
}

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
    brio::print(serial, brio::crlf, "wire_check STM32F446RE", brio::crlf);

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

extern "C" void USART2_IRQHandler() { Serial::isr(); }
