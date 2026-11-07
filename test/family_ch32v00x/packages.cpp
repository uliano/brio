// The package axis, every package: each driver that takes pads compiled
// over pads THIS package carries - the first column of each remap table
// the package bonds whole, a timer pad, an analog input, an external
// interrupt line - so that a package whose pins leave out the columns the
// other fixtures name (they pin the CH32V006's QFN32 and the CH32V003's
// TSSOP20) still compiles every driver it can carry. Where a package
// carries no column of a peripheral (the CH32V003's SOP8 has no USART TX
// alone on its pin), nothing is instantiated and the search's answer is
// pinned instead. The refusals of absent and shared pads are the neg/
// TUs'.
#include "ch32v00x/adc.hpp"
#include "ch32v00x/clock.hpp"
#include "ch32v00x/exti.hpp"
#include "ch32v00x/i2c.hpp"
#include "ch32v00x/opa.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/spi.hpp"
#include "ch32v00x/tim.hpp"
#include "ch32v00x/usart.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

// ---- the searches: the first column, pad or channel the package carries ----
constexpr int usart1_column() {
    for (uint8_t c = 0; c < afio_usart1_codes; ++c) {
        if (usart_pads_bonded(1, c)) { return c; }
    }
    return -1;
}
constexpr int usart2_column() {
    for (uint8_t c = 1; c < afio_usart2_codes; ++c) {
        if (usart_pads_bonded(2, c)) { return c; }
    }
    return -1;
}
/// A full-duplex host's three pads, no NSS (the select is a GPIO).
constexpr SpiPins spi_link(uint8_t c) {
    const SpiPins p = spi1_pins_for(c);
    return SpiPins{.sck = p.sck, .mosi = p.mosi, .miso = p.miso, .remap = c};
}
constexpr int spi1_column() {
    for (uint8_t c = 0; c < afio_spi1_codes; ++c) {
        const SpiPins p = spi_link(c);
        if (spi_pins_bonded(p) && pad_can_drive(p.sck) && pad_can_drive(p.mosi)) { return c; }
    }
    return -1;
}
constexpr int i2c1_column() {
    for (uint8_t c = 0; c < afio_i2c1_codes; ++c) {
        if (i2c_pins_bonded(i2c1_pins_for(c))) { return c; }
    }
    return -1;
}
constexpr Pad tim1_ch1_pad() {
    for (uint8_t c = 0; c < afio_tim1_codes; ++c) {
        if (!afio_tim1_ch1_is_lsi(c) && pad_can_drive(afio_tim1_pads(c).ch1)) { return afio_tim1_pads(c).ch1; }
    }
    return Pad{};
}
constexpr Pad analog_pad() {
    for (char port = 'A'; port <= 'D'; ++port) {
        for (uint8_t pin = 0; pin < 8u; ++pin) {
            if (pad_bonded(Pad{port, pin}) && adc_channel_of(Pad{port, pin}) != 0xFFu) { return Pad{port, pin}; }
        }
    }
    return Pad{};
}
constexpr Pad any_pad() {
    for (char port = 'A'; port <= 'D'; ++port) {
        for (uint8_t pin = 0; pin < 8u; ++pin) {
            if (pad_bonded(Pad{port, pin})) { return Pad{port, pin}; }
        }
    }
    return Pad{};
}

// What the searches find, pinned where the datasheet's tables decide it:
// the bench packages keep their default columns, the CH32V006's QFN12
// moves USART1 to column 2 (PD0/PD1), the CH32V003's SOP8 has no USART
// and no SPI column it may drive.
static_assert(device::package != Ch32Package::pins32 ||
              (usart1_column() == 0 && spi1_column() == 0 && i2c1_column() == 0));
static_assert(device::part != Ch32Part::v006 || device::package != Ch32Package::pins20 ||
              (usart1_column() == 0 && usart2_column() == 3 && spi1_column() == 0 && i2c1_column() == 0));
static_assert(device::part != Ch32Part::v006 || device::package != Ch32Package::pins12 || usart1_column() == 2);
static_assert(device::package != Ch32Package::pins8 || (usart1_column() < 0 && spi1_column() < 0));
static_assert(any_pad().valid() && analog_pad().valid());

// ---- each driver over the pads found, instantiated where there are some ---
template <bool present, int column>
struct UartCase {
    static void run() {}
};
template <int column>
struct UartCase<true, column> {
    using U1 = Uart<1, P, 16, 16, NoDmaEngine, NoDmaEngine, static_cast<uint8_t>(column)>;
    static void run() {
        constexpr SysClock clock;
        (void)U1::init(clock, 115200);
        (void)U1::write_byte(0x55);
        (void)U1::isr();
    }
};

template <bool present, int column>
struct Uart2Case {
    static void run() {}
};
template <int column>
struct Uart2Case<true, column> {
    using U2 = Uart<2, P, 16, 16, NoDmaEngine, NoDmaEngine, static_cast<uint8_t>(column)>;
    static void run() {
        constexpr SysClock clock;
        (void)U2::init(clock, 9600);
        (void)U2::isr();
    }
};

template <bool present, int column>
struct SpiCase {
    static void run() {}
};
template <int column>
struct SpiCase<true, column> {
    using H = SpiHost<1, spi_link(static_cast<uint8_t>(column))>;
    static void run() {
        constexpr SysClock clock;
        (void)H::init(clock, 4'000'000);
        H::release();
    }
};

template <bool present, int column>
struct I2cCase {
    static void run() {}
};
template <int column>
struct I2cCase<true, column> {
    using H = I2cHost<1, i2c1_pins_for(static_cast<uint8_t>(column))>;
    static void run() {
        constexpr SysClock clock;
        (void)H::init(clock);
        (void)H::isr();
    }
};

template <bool present, Pad pad>
struct TimCase {
    static void run() {}
};
template <Pad pad>
struct TimCase<true, pad> {
    static void run() {
        TimPad<pad>::claim();
        TimPad<pad>::release();
    }
};

void package_drivers() {
    UartCase<(usart1_column() >= 0), usart1_column()>::run();
    if constexpr (device::has_usart2) {
        Uart2Case<(usart2_column() >= 0), usart2_column()>::run();
    }
    SpiCase<(spi1_column() >= 0), spi1_column()>::run();
    I2cCase<(i2c1_column() >= 0), i2c1_column()>::run();
    TimCase<tim1_ch1_pad().valid(), tim1_ch1_pad()>::run();

    using In = AnalogIn<Pin<analog_pad().port, analog_pad().pin>>;
    In::claim();
    using Line = ExtInt<any_pad().port, any_pad().pin>;
    Line::Input::input(PinPull::up);
    Line::init(true, false);

    // The OPA's default config is valid exactly where its output pad is
    // the package's to drive (opa.hpp).
    (void)Opa::configure(OpaConfig{});
}
