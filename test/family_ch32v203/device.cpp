// The RESERVE itself: every fact parts/<part>.hpp states, checked for
// internal consistency and against the register map that reads it. This
// is the one TU whose subject is the table rather than a driver, and it
// is where the family's three irregularities are pinned - the usart list
// that is not the first n instances, the packages with no oscillator
// pads, and the device class that moves two vector numbers.
#include "ch32v203/device.hpp"

using namespace brio;

// ---- the memories, in the three tiers the datasheet's table 2-1 gives -------
static_assert(device::flash_bytes == 32u * 1024u || device::flash_bytes == 64u * 1024u ||
              device::flash_bytes == 128u * 1024u);
static_assert(device::sram_bytes == 10u * 1024u || device::sram_bytes == 20u * 1024u ||
              device::sram_bytes == 64u * 1024u);
// The tiers go together: 32K with 10K, 64K with 20K, 128K with 64K.
static_assert((device::flash_bytes == 32u * 1024u) == (device::sram_bytes == 10u * 1024u));
static_assert((device::flash_bytes == 64u * 1024u) == (device::sram_bytes == 20u * 1024u));
static_assert((device::flash_bytes == 128u * 1024u) == (device::sram_bytes == 64u * 1024u));
static_assert(device::flash_page_bytes == 256u);
static_assert(device::flash_protect_bytes == 4096u);

// ---- the class, and what follows from it -----------------------------------
// The 128 KB part is the D8 and no other; the class decides the vector
// table's length and the index of the two lines the Ethernet pair
// displaces.
static_assert(device::is_d8_class == (device::flash_bytes == 128u * 1024u));
static_assert(device::vector_count == (device::is_d8_class ? 70u : 63u));
static_assert(static_cast<uint8_t>(Irq::uart4) == (device::is_d8_class ? 66 : 61));
static_assert(static_cast<uint8_t>(Irq::dma1_channel8) == (device::is_d8_class ? 67 : 62));
static_assert(static_cast<uint8_t>(Irq::uart4) < device::vector_count);
static_assert(static_cast<uint8_t>(Irq::dma1_channel8) < device::vector_count);
// Everything below the tail is the family's and moves for nobody.
static_assert(static_cast<uint8_t>(Irq::systick) == 12);
static_assert(static_cast<uint8_t>(Irq::usart1) == 53);
static_assert(static_cast<uint8_t>(Irq::usbfs) == 59);
static_assert(static_cast<uint8_t>(Irq::usb_wakeup) == 58);

// ---- the pads --------------------------------------------------------------
// Port A always reaches a pad, port E never does on any part of the
// series, and a port with no pin is no port.
static_assert(device::has_port('A'));
static_assert(!device::has_port('E'));
static_assert(device::port_pins('E') == 0u);
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
    for (uint8_t i = 1; i <= 4; ++i) {
        if (device::has_usart(i)) {
            ++n;
        }
    }
    return n;
}
static_assert(counted_usarts() == device::usart_count);
static_assert(device::usart_count >= 1u && device::usart_count <= 4u);
// USART2 is the one instance EVERY part of the family offers - which is
// what the smallest package proves, having no other.
static_assert(device::has_usart(2));
static_assert(!device::has_usart(0) && !device::has_usart(5));
// Four instances means all four; two means the first two.
static_assert(device::usart_count != 4u || (device::has_usart(3) && device::has_usart(4)));
static_assert(device::usart_count != 2u || device::has_usart(1));
// A part that offers an instance bonds its default pads, and one that
// does not may still bond them (the die is what the count follows).
static_assert(!device::has_usart(1) || (device::port_pins('A') & (1u << 9)) != 0u);

constexpr uint8_t counted_opas() {
    uint8_t n = 0;
    for (uint8_t i = 1; i <= 2; ++i) {
        if (device::has_opa(i)) {
            ++n;
        }
    }
    return n;
}
static_assert(counted_opas() == device::opa_count);
static_assert(device::has_opa(2));   // as with the usart, the second is the survivor

static_assert(device::spi_count >= 1u && device::spi_count <= 2u);
static_assert(device::i2c_count <= 2u);
static_assert(device::can_count <= 1u);
static_assert(device::adc_count == (device::is_d8_class ? 1u : 2u));
static_assert(device::adc_channel_count == 9u || device::adc_channel_count == 10u ||
              device::adc_channel_count == 16u);
// The channel count is the bonding: channels 8 and 9 are PB0 and PB1,
// 10..15 are PC0..PC5.
static_assert(device::adc_channel_count != 16u || (device::port_pins('C') & 0x003Fu) == 0x003Fu);
static_assert(device::advanced_timer_count == 1u && device::general_timer_count == 3u);
static_assert(device::has_tim5 == device::is_d8_class);
static_assert(device::has_ethernet == device::is_d8_class);
// One package of the series has no device controller; every part that
// has one bonds PA11 and PA12.
static_assert(!device::has_usbd || (device::port_pins('A') & 0x1800u) == 0x1800u);
static_assert(!device::has_usbfs || (device::port_pins('B') & 0x00C0u) == 0x00C0u);

// ---- the clock tree's edges ------------------------------------------------
static_assert(device::sysclk_max_hz == 144'000'000UL);
static_assert(device::hse_min_hz <= device::hse_max_hz);
static_assert(device::hse_max_hz == (device::is_d8_class ? 32'000'000UL : 25'000'000UL));
static_assert(!device::is_d8_class || device::hse_min_hz == device::hse_max_hz);

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
static_assert(usart_base_for(5) == 0);
static_assert(hsi_hz == 8'000'000UL);
// The low-speed RC is a PART fact: the CH32V203RB's is the 32 kHz one.
static_assert(device::lsi_min_hz < device::lsi_typ_hz && device::lsi_typ_hz < device::lsi_max_hz);
static_assert(device::lsi_typ_hz == (device::is_d8_class ? 32'000UL : 39'000UL));
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
