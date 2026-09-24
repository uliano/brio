// The RESERVE itself: every fact parts/<part>.hpp states, checked for
// internal consistency and against the register map that reads it. This
// is the one TU whose subject is the table rather than a driver, and it
// is where the family's irregularities are pinned - the usart list that
// is not the first n instances, the packages with no oscillator pads,
// and the three device classes with three vector tails, of which the
// CH32V303's third adds a second DMA controller and the floating-point
// unit.
#include "ch32v203/device.hpp"

#include <bit>

using namespace brio;

inline constexpr bool d6 = device::device_class == DeviceClass::v20x_d6;
inline constexpr bool d8 = device::device_class == DeviceClass::v20x_d8;
inline constexpr bool v30x = device::device_class == DeviceClass::v30x_d8;

// ---- the memories, in the tiers the two datasheets' tables give ------------
static_assert(device::flash_bytes == 32u * 1024u || device::flash_bytes == 64u * 1024u ||
              device::flash_bytes == 128u * 1024u || device::flash_bytes == 256u * 1024u);
static_assert(device::sram_bytes == 10u * 1024u || device::sram_bytes == 20u * 1024u ||
              device::sram_bytes == 32u * 1024u || device::sram_bytes == 64u * 1024u);
// The tiers go together: on the CH32V203 32K with 10K, 64K with 20K and
// 128K with 64K; on the CH32V303 128K with 32K and 256K with 64K.
static_assert(v30x || (device::flash_bytes == 32u * 1024u) == (device::sram_bytes == 10u * 1024u));
static_assert(v30x || (device::flash_bytes == 64u * 1024u) == (device::sram_bytes == 20u * 1024u));
static_assert(v30x || (device::flash_bytes == 128u * 1024u) == (device::sram_bytes == 64u * 1024u));
static_assert(!v30x || (device::flash_bytes == 128u * 1024u) == (device::sram_bytes == 32u * 1024u));
static_assert(!v30x || (device::flash_bytes == 256u * 1024u) == (device::sram_bytes == 64u * 1024u));
static_assert(device::flash_page_bytes == 256u);
static_assert(device::flash_protect_bytes == 4096u);
// The window, the tail and the array: 224 KB on every CH32V203, the
// window's complement above it; 480 KB on every CH32V303, with a tail
// only where the datasheet grants one (the 256 KB parts).
static_assert(device::flash_bytes + device::flash_tail_bytes <= device::flash_array_bytes);
static_assert(device::flash_array_bytes == (v30x ? 480u : 224u) * 1024u);
static_assert(v30x || device::flash_bytes + device::flash_tail_bytes == device::flash_array_bytes);
static_assert(!v30x || device::flash_tail_bytes ==
                           (device::flash_bytes == 256u * 1024u ? 224u * 1024u : 0u));

// ---- the class, and what follows from it -----------------------------------
// The 128 KB CH32V203 is the CH32V20x_D8 and no other; every CH32V303 is
// the CH32V30x_D8, and it alone has the floating-point unit. The class
// decides the vector table's length and where each line of its tail
// sits.
static_assert(d8 == (!v30x && device::flash_bytes == 128u * 1024u));
static_assert(device::has_fpu == v30x);
static_assert(device::vector_count == (d6 ? 63u : d8 ? 70u : 104u));
static_assert(static_cast<uint8_t>(Irq::uart4) == (d6 ? 61 : d8 ? 66 : 68));
static_assert(static_cast<uint8_t>(Irq::dma1_channel8) == (d6 ? 62 : d8 ? 67 : irq_none));
static_assert(static_cast<uint8_t>(Irq::usbfs) == (v30x ? 83 : 59));
static_assert(static_cast<uint8_t>(Irq::usb_wakeup) == 58);
static_assert(static_cast<uint8_t>(Irq::usbfs_wakeup) == (v30x ? 84 : 60));
static_assert(static_cast<uint8_t>(Irq::uart4) < device::vector_count);
static_assert(static_cast<uint8_t>(Irq::dma1_channel8) < device::vector_count);
static_assert(static_cast<uint8_t>(Irq::dma2_channel11) < device::vector_count);
// The last word of each tail is the table's last entry.
static_assert(!d6 || static_cast<uint8_t>(Irq::dma1_channel8) + 1u == device::vector_count);
static_assert(!d8 || static_cast<uint8_t>(Irq::osc32k_wakeup) + 1u == device::vector_count);
static_assert(!v30x || static_cast<uint8_t>(Irq::dma2_channel11) + 1u == device::vector_count);
// The CH32V303's own lines, and a line a class has not got is none.
static_assert(!v30x || (static_cast<uint8_t>(Irq::tim8_brk) == 59 &&
                        static_cast<uint8_t>(Irq::rng) == 63 &&
                        static_cast<uint8_t>(Irq::tim5) == 66 &&
                        static_cast<uint8_t>(Irq::uart5) == 69 &&
                        static_cast<uint8_t>(Irq::dma2_channel1) == 72 &&
                        static_cast<uint8_t>(Irq::uart6) == 87 &&
                        static_cast<uint8_t>(Irq::tim10_cc) == 97 &&
                        static_cast<uint8_t>(Irq::dma2_channel6) == 98));
static_assert(v30x || !irq_exists(Irq::tim8_brk));
static_assert(!d8 || static_cast<uint8_t>(Irq::tim5) == 65);
static_assert(!d6 || !irq_exists(Irq::tim5));
static_assert(irq_present<Irq::usart1>() == Irq::usart1);
// Everything below the tail is the family's and moves for nobody.
static_assert(static_cast<uint8_t>(Irq::systick) == 12);
static_assert(static_cast<uint8_t>(Irq::usart1) == 53);
static_assert(static_cast<uint8_t>(Irq::rtc_alarm) == 57);
static_assert(irq_none == 0u);

// ---- the DMA controllers ----------------------------------------------------
// One of eight channels on the CH32V203; two on the CH32V303, of seven
// and eleven.
static_assert(device::dma_controller_count == (v30x ? 2u : 1u));
static_assert(device::dma1_channel_count == (v30x ? 7u : 8u));
static_assert(device::dma2_channel_count == (device::dma_controller_count == 2u ? 11u : 0u));

// ---- the pads --------------------------------------------------------------
// Port A always reaches a pad, port E only on the LQFP100 - the one part
// with an FSMC to bring out - and a port with no pin is no port.
static_assert(device::has_port('A'));
static_assert(device::has_port('E') == device::has_fsmc);
static_assert(device::has_port('E') || device::port_pins('E') == 0u);
static_assert(device::has_port('B'));
static_assert(device::has_port('C') == (device::port_pins('C') != 0u));
// PA0..PA7 are bonded on every package (they are ADC_IN0..7), and so are
// the two debug pads.
static_assert((device::port_pins('A') & 0x00FFu) == 0x00FFu);
static_assert((device::port_pins(device::debug_swdio_port) & (1u << device::debug_swdio_pin)) != 0u);
static_assert((device::port_pins(device::debug_swclk_port) & (1u << device::debug_swclk_pin)) != 0u);
// The oscillator pads and port D are the same fact on every part but the
// largest, where OSC_IN and OSC_OUT cannot be handed back to port D.
static_assert(device::has_hse_pins || device::port_pins('D') == 0u);

// ---- the instances ---------------------------------------------------------
// The mask and the count say the same thing, and the count is the
// datasheet's column.
constexpr uint8_t counted_usarts() {
    uint8_t n = 0;
    for (uint8_t i = 1; i <= 8; ++i) {
        if (device::has_usart(i)) {
            ++n;
        }
    }
    return n;
}
static_assert(counted_usarts() == device::usart_count);
static_assert(device::usart_count >= 1u && device::usart_count <= 8u);
// USART2 is the one instance EVERY part of the family offers - which is
// what the smallest package proves, having no other.
static_assert(device::has_usart(2));
static_assert(!device::has_usart(0) && !device::has_usart(9));
// Four instances means all four; two means the first two; three the
// three full ones; eight all eight, the CH32V303RC's and VC's.
static_assert(device::usart_count != 4u || (device::has_usart(3) && device::has_usart(4)));
static_assert(device::usart_count != 2u || device::has_usart(1));
static_assert(device::usart_count != 3u || (device::has_usart(1) && device::has_usart(3)));
static_assert(device::has_usart(5) == (device::usart_count == 8u));
static_assert(device::usart_count <= 4u || v30x);
// The full ones are USART1..3 everywhere but the CH32V203C8, whose
// fourth is a USART4 as well.
static_assert((device::usart_full_instances & ~device::usart_instances) == 0u);
static_assert((device::usart_full_instances & 0xFFE0u) == 0u);
// A part that offers an instance bonds its default pads, and one that
// does not may still bond them (the die is what the count follows).
static_assert(!device::has_usart(1) || (device::port_pins('A') & (1u << 9)) != 0u);

constexpr uint8_t counted_opas() {
    uint8_t n = 0;
    for (uint8_t i = 1; i <= 4; ++i) {
        if (device::has_opa(i)) {
            ++n;
        }
    }
    return n;
}
static_assert(counted_opas() == device::opa_count);
static_assert(device::has_opa(2));   // as with the usart, the second is the survivor
static_assert(device::opa_count == (v30x ? 4u : device::opa_count));
static_assert(v30x || device::opa_count <= 2u);

// The synchronous ports: the count is the mask's, SPI1 is in every one,
// and the I2S face goes with the third.
static_assert(device::spi_count >= 1u && device::spi_count <= 3u);
static_assert(std::popcount(device::spi_instances) == device::spi_count);
static_assert((device::spi_instances & (1u << 1)) != 0u);
static_assert(device::has_i2s == (device::spi_count == 3u));
static_assert(device::spi_count <= 2u || v30x);
static_assert(device::i2c_count <= 2u);
static_assert(device::can_count <= 1u);
static_assert(device::adc_count == (d8 ? 1u : 2u));
static_assert(device::adc_channel_count == 9u || device::adc_channel_count == 10u ||
              device::adc_channel_count == 16u);
// The channel count is the bonding: channels 8 and 9 are PB0 and PB1,
// 10..15 are PC0..PC5.
static_assert(device::adc_channel_count != 16u || (device::port_pins('C') & 0x003Fu) == 0x003Fu);
// The timers: the counts are the masks' (the CH32V203RB's 32-bit TIM5
// is counted apart from its three sixteen-bit ones, as its datasheet
// counts it), TIM1 is in every part and the basic pair goes with the
// eight serial ports.
static_assert(std::popcount(device::advanced_timer_instances) == device::advanced_timer_count);
static_assert(std::popcount(device::general_timer_instances) ==
              device::general_timer_count + (d8 ? 1 : 0));
static_assert((device::advanced_timer_instances & (1u << 1)) != 0u);
static_assert(device::has_tim5 == ((device::general_timer_instances & (1u << 5)) != 0u));
static_assert(device::basic_timer_instances == 0u ||
              device::basic_timer_instances == ((1u << 6) | (1u << 7)));
static_assert((device::basic_timer_instances != 0u) == (device::usart_count == 8u));
static_assert(v30x || (device::advanced_timer_count == 1u && device::general_timer_count == 3u));
static_assert(device::has_tim5 == (d8 || (v30x && device::flash_bytes == 256u * 1024u)));
static_assert(device::has_ethernet == d8);
// The blocks the CH32V303 adds: the DACs on all four, the RNG and the
// SDIO host on the 256 KB two, the FSMC on the LQFP100.
static_assert(device::has_dac == v30x);
static_assert(device::has_rng == device::has_sdio);
static_assert(device::has_rng == (v30x && device::flash_bytes == 256u * 1024u));
static_assert(!device::has_fsmc || device::has_rng);
// One package of the series has no device controller; every part that
// has one bonds PA11 and PA12.
static_assert(!device::has_usbd || (device::port_pins('A') & 0x1800u) == 0x1800u);
static_assert(!device::has_usbfs || (device::port_pins('B') & 0x00C0u) == 0x00C0u);

// ---- the clock tree's edges ------------------------------------------------
static_assert(device::sysclk_max_hz == 144'000'000UL);
static_assert(device::hse_min_hz <= device::hse_max_hz);
static_assert(device::hse_max_hz == (d8 ? 32'000'000UL : 25'000'000UL));
static_assert(!d8 || device::hse_min_hz == device::hse_max_hz);
// The PLL's own ceiling is 144 MHz but on the CH32V203RB, whose 240 MHz
// feeds the USB divider's fifth code, and that code is its alone.
static_assert(device::pll_out_max_hz == (d8 ? 240'000'000UL : 144'000'000UL));
static_assert(device::has_usb_pre_div5 == d8);
static_assert(device::pll_hse_div[0] == (d8 ? 4u : 1u));
// The HSE reaches the RTC divided by 128 or 512: a lot decides it on the
// D6, the class alone on the other two.
static_assert(device::rtc_hse_div[0] == (v30x ? 128u : 512u));
static_assert(device::rtc_hse_div[1] == (d8 ? 512u : 128u));
static_assert(device::bkp_data_registers == (d6 ? 10u : 42u));

// ---- the map that reads the table ------------------------------------------
static_assert(pb1_base == 0x40000000UL && pb2_base == 0x40010000UL);
static_assert(hb_base == 0x40020000UL && usbfs_base == 0x50000000UL);
static_assert(gpio_base_for('A') == pb2_base + 0x0800);
static_assert(gpio_base_for('E') == pb2_base + 0x1800);
static_assert(gpio_base_for('F') == 0);
static_assert(gpio_clock_for('C') == rcc_pb2_gpioc);
static_assert(gpio_clock_for('F') == 0);
static_assert(usart_base_for(1) == pb2_base + 0x3800);
static_assert(usart_base_for(4) == pb1_base + 0x4c00);
static_assert(usart_base_for(5) == pb1_base + 0x5000 && usart_base_for(8) == pb1_base + 0x2000);
static_assert(usart_base_for(9) == 0);
// The blocks the CH32V303 adds, where RM figure 1-13 puts them.
static_assert(dma2_base == 0x40020400UL && rng_base == 0x40023C00UL);
static_assert(tim6_base == 0x40001000UL && tim7_base == 0x40001400UL);
static_assert(spi3_base == 0x40003C00UL && dac_base == 0x40007400UL);
static_assert(tim8_base == 0x40013400UL && tim9_base == 0x40014C00UL);
static_assert(tim10_base == 0x40015000UL && sdio_base == 0x40018000UL);
static_assert(fsmc_bank1_base == 0x60000000UL && fsmc_regs_base == 0xA0000000UL);
static_assert(rcc_hb_dma2 == (1u << 1) && rcc_hb_rng == (1u << 9) && rcc_hb_sdio == (1u << 10));
static_assert(rcc_pb1_dac == (1u << 29) && rcc_pb1_uart5 == (1u << 20) && rcc_pb1_spi3 == (1u << 15));
static_assert(rcc_pb2_tim8 == (1u << 13) && rcc_pb2_tim10 == (1u << 20));
static_assert(exten2_opa_hsmd(1) == 1u && exten2_opa_hsmd(4) == 8u);
static_assert(hsi_hz == 8'000'000UL);
// The low-speed RC is a PART fact: the CH32V203RB's is the 32 kHz one.
static_assert(device::lsi_min_hz < device::lsi_typ_hz && device::lsi_typ_hz < device::lsi_max_hz);
static_assert(device::lsi_typ_hz == (d8 ? 32'000UL : 39'000UL));
static_assert(flash_array_base == 0x08000000UL && flash_alias_base == 0x00000000UL);
static_assert(flash_key1 == 0x45670123UL && flash_key2 == 0xCDEF89ABUL);

// The PLL ladder is x2..x16 and then x18, which is the one code that is
// not its index plus two.
static_assert(rcc_pllmul_code(2) == 0u && rcc_pllmul_of(0) == 2u);
static_assert(rcc_pllmul_code(16) == 14u && rcc_pllmul_of(14) == 16u);
static_assert(rcc_pllmul_code(18) == 15u && rcc_pllmul_of(15) == 18u);

// The register views the drivers reach through.
void device_views() {
    (void)rcc();
    (void)afio();
    (void)flash_ctl();
    (void)exten();
    (void)stk();
    (void)pfic();
    (void)&pfic_sctlr();
    (void)esig_flash_kbytes();
    (void)esig_uid_word(0);
}
