// AFIO family smoke TU: ch32v00x/afio.hpp's tables and verbs, and the
// drivers taking their pads from them - instantiation only, no main(),
// no hardware. The tables' corners are pinned in the header; what this
// fixture adds is that every driver accepts a remapped pin set.
#include "ch32v00x/afio.hpp"
#include "ch32v00x/clock.hpp"
#if BRIO_CH32_PART_V006
#include "ch32v00x/i2c.hpp"
#endif
#include "ch32v00x/platform.hpp"
#include "ch32v00x/spi.hpp"
#include "ch32v00x/tim.hpp"
#include "ch32v00x/usart.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

// Every column of every table has valid pads.
constexpr bool all_valid() {
    for (uint8_t c = 0; c < afio_tim1_codes; ++c) {
        const Tim1Pads t = afio_tim1_pads(c);
        if (!(t.etr.valid() && t.ch1.valid() && t.ch2.valid() && t.ch3.valid() && t.ch4.valid() &&
              t.bkin.valid() && t.ch1n.valid() && t.ch2n.valid() && t.ch3n.valid())) {
            return false;
        }
    }
    for (uint8_t c = 0; c < afio_tim2_codes; ++c) {
        const Tim2Pads t = afio_tim2_pads(c);
        if (!(t.ch1_etr.valid() && t.ch2.valid() && t.ch3.valid() && t.ch4.valid())) {
            return false;
        }
    }
    for (uint8_t c = 0; c < afio_usart1_codes; ++c) {
        const UsartPadSet u = afio_usart1_pads(c);
        if (!(u.tx.valid() && u.rx.valid() && u.cts.valid() && u.rts.valid())) {
            return false;
        }
    }
    for (uint8_t c = 0; c < afio_spi1_codes; ++c) {
        const SpiPadSet u = afio_spi1_pads(c);
        if (!(u.nss.valid() && u.sck.valid() && u.miso.valid() && u.mosi.valid())) {
            return false;
        }
    }
    return true;
}
static_assert(all_valid());

// Column 3 of USART1 and TIM1 is PC0 and PC4 on both parts; the SPI
// and I2C columns are each part's.
#if BRIO_CH32_PART_V006
using SpiRemapped = SpiHost<1, spi1_pins_for(2)>;
using I2cRemapped = I2cHost<1, i2c1_pins_for(1)>;
static_assert(SpiRemapped::pin_pads.sck == Pad{'D', 2});
static_assert(I2cRemapped::pin_pads.scl == Pad{'D', 1});
#else
using SpiRemapped = SpiHost<1, spi1_pins_for(1)>;
static_assert(SpiRemapped::pin_pads.nss == Pad{'C', 0});
#endif
using UartRemapped = Uart<1, P, 64, 64, NoDmaEngine, NoDmaEngine, 3>;
using Tim1Ch1Remapped = TimPad<afio_tim1_pads(3).ch1>;   // PC4

static_assert(std::same_as<UartRemapped::Tx, Pin<'C', 0>>);
static_assert(Tim1Ch1Remapped::selection == Pad{'C', 4});

void verbs() {
    constexpr SysClock clock;
    Afio::clock_on();
    Afio::remap_tim1(3); (void)Afio::tim1_remap();
    Afio::remap_tim2(afio_tim2_codes - 1u); (void)Afio::tim2_remap();
    Afio::tim1_ch1_from_lsi(true); Afio::tim1_ch1_from_lsi(false);
    Afio::remap_usart1(2); (void)Afio::usart1_remap();
    Afio::remap_usart2(3); (void)Afio::usart2_remap();
    Afio::remap_spi1(afio_spi1_codes - 1u); (void)Afio::spi1_remap();
    Afio::remap_i2c1(1); (void)Afio::i2c1_remap();
    Afio::remap_adc_injected_trigger(true);
    Afio::remap_adc_rule_trigger(false);
    Afio::pa1_pa2_gpio(true);
    (void)Afio::debug_port_enabled();
    (void)SpiRemapped::init(clock);
#if BRIO_CH32_PART_V006
    (void)I2cRemapped::init(clock);
#endif
    (void)UartRemapped::init(clock, 9600);
    (void)Tim<1>::remap(3); (void)Tim<1>::remap();
    (void)Tim<2>::remap(afio_tim2_codes - 1u); (void)Tim<2>::remap();
    Tim1Ch1Remapped::claim();
}
