/*
 * parts/ch32v203c6.hpp
 *
 * The CH32V203C6 as a table of facts: 32 KB of flash, 10 KB of SRAM,
 * the CH32V203C8's LQFP48 pinout exactly, and WCH's CH32V20x_D6 device
 * class. The rules this file obeys are parts/ch32v203c8.hpp's; read
 * that one first.
 *
 * THE SAME PACKAGE, THE SMALLER DIE. Pin for pin this part is the C8 -
 * one pin table (datasheet 3-1-1) covers both - so what separates them
 * is not bonding but silicon: table 2-1 gives the C6 two USARTs where
 * the C8 has four, one SPI where it has two and one I2C where it has
 * two, and those pads (PB10/PB11, PB0/PB1, PB12..PB15, PB6/PB7) are all
 * brought out here. A driver that inferred an instance from a pad would
 * be wrong on exactly this part.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

/// The name this part answers to on a console banner.
inline constexpr const char* part_name = "CH32V203C6";

// ---- the memories (datasheet table 2-1) -----------------------------------
inline constexpr uint32_t flash_bytes = 32UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 10UL * 1024UL;
/// The flash's grain: 256 bytes, the unit of the fast program and erase
/// verbs (RM 32.1), with write protection in 4 KB units above it.
inline constexpr uint32_t flash_page_bytes      = 256UL;
inline constexpr uint32_t flash_protect_bytes   = 4096UL;
/// The code flash is one array of 224 KB on every part of this series,
/// and flash_bytes above is only its ZERO-WAIT WINDOW: the datasheet's
/// note 1 to table 2-1 counts "Flash bytes" as the zero-wait run area
/// and gives the V203 series a non-zero-wait area of (224K - R0WAIT)
/// above it. The tail is stated and nothing uses it: no linker script
/// places a byte there and what an access there costs is not measured.
inline constexpr uint32_t flash_tail_bytes  = 192UL * 1024UL;
inline constexpr uint32_t flash_array_bytes = 224UL * 1024UL;

// ---- the device class -----------------------------------------------------
/// WCH's own family division, which several chapters are keyed by: this
/// part is CH32V20x_D6 (RM, "Specific classification abbreviations").
/// A register description that names another class is about other
/// silicon; a driver asks the facts below and names the class only
/// where the manual's note is a list of classes.
inline constexpr DeviceClass device_class = DeviceClass::v20x_d6;
/// The core: the QingKe V4B, RV32IMAC with no floating-point unit.
inline constexpr bool has_fpu = false;
/// The vector table's length, which follows the class: 63 entries, the
/// last being the eighth DMA channel (startup_ch32vx03.S).
inline constexpr uint32_t vector_count = 63;

/// The backup domain's DATA registers, which the DEVICE CLASS decides:
/// ten here - BKP_DATAR1..BKP_DATAR10, twenty bytes - where the
/// CH32V20x_D8 carries forty-two (RM 4.3's note under table 4-1). The
/// block's other registers (the calibration, the tamper pair) are every
/// class's.
inline constexpr uint8_t bkp_data_registers = 10;

/// What RTCSEL's third choice divides the HSE by on the way to the RTC
/// (RM 3.4.9). On this device class it is ONE OF TWO NUMBERS and the
/// register description keys them on the LOT: 512 where the penultimate
/// fifth digit of the lot number is less than one, 128 otherwise. Both
/// are stated because neither is knowable from the part number, and a
/// program that clocks the RTC from the crystal measures which of them
/// this die has; where the manual states a single value, the two
/// entries are equal.
inline constexpr uint32_t rtc_hse_div[2] = {512, 128};

// ---- the DMA controllers (RM 11.3's register notes) -----------------------
/// One controller here, with EIGHT channels: RM 11.3.1's notes give the
/// eighth to this device class among the five that have it.
inline constexpr uint8_t dma_controller_count = 1;
inline constexpr uint8_t dma1_channel_count   = 8;
inline constexpr uint8_t dma2_channel_count   = 0;

// ---- the pads (datasheet 3.2, table 3-1-1's LQFP48 column) ----------------
/// Which pins each port bonds, as a mask. Thirty-seven in all: two whole
/// ports, three pins of port C and the two of port D that the package
/// brings out (the same two pads the oscillator uses, which is why they
/// are pins 5 and 6 of this package and the datasheet's note 4 is about
/// them). Port E is not bonded on any part of this series.
inline constexpr uint16_t port_pins(char port) {
    return port == 'A' ? 0xFFFFU :
           port == 'B' ? 0xFFFFU :
           port == 'C' ? 0xE000U :   // PC13, PC14, PC15
           port == 'D' ? 0x0003U :   // PD0, PD1
           0x0000U;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0U; }

/// Whether AFIO's PD0PD1_RM hands the oscillator's two pads to port D as
/// PD0 and PD1 (RM 10.2.11.2): yes - the section names the CH32V203CxT6
/// and CxU6 among the parts whose OSC_IN/OSC_OUT can be handed to port D.
inline constexpr bool osc_pads_as_pd0_pd1 = true;

/// The two pads the debug port owns from reset: a program that takes
/// them for itself loses the probe until the next power cycle.
inline constexpr char debug_swdio_port = 'A';
inline constexpr uint8_t debug_swdio_pin = 13;
inline constexpr char debug_swclk_port = 'A';
inline constexpr uint8_t debug_swclk_pin = 14;

// ---- the instances (datasheet table 2-1) ----------------------------------
/// The USARTs this part offers, one bit per instance number: two of
/// them, and they are USART1 and USART2 - USART3's pads (PB10/PB11) and
/// UART4's (PB0/PB1) are bonded on this package and answer nothing, so
/// the count here is the DIE's and not the package's.
inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2);
inline constexpr uint8_t usart_count = 2;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// Which of those instances are FULL USARTs - the synchronous clock on
/// a CK pad, the smartcard and the flow-control pair - rather than
/// asynchronous receivers alone (RM 18, its opening note). Every
/// instance this part offers is one: the family's UART4 is the only
/// serial port that is not, and this package has not got it.
inline constexpr uint16_t usart_full_instances = usart_instances;

inline constexpr bool usart_full(uint8_t n) {
    return n < 16U && (usart_full_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t spi_count = 1;
/// The synchronous ports as a mask, one bit per instance number: SPI1 alone
/// here. The I2S face of SPI2 and SPI3 is the CH32V303RC's and VC's;
/// this series has none (datasheet table 2-1 has no I2S row).
inline constexpr uint16_t spi_instances = (1U << 1);
inline constexpr bool has_i2s = false;
inline constexpr uint8_t i2c_count = 1;
inline constexpr uint8_t can_count = 1;
/// The converters, and how many of the sixteen analog channels this
/// package brings out to a pad (ADC_IN0..7 are PA0..PA7, 8 and 9 are
/// PB0 and PB1, 10..15 are PC0..PC5 - datasheet 3.3).
inline constexpr uint8_t adc_count         = 2;
inline constexpr uint8_t adc_channel_count = 10;

/// The operational amplifiers, one bit per instance number: both, each
/// with an input pair and an output on a bonded pad.
inline constexpr uint16_t opa_instances = (1U << 1) | (1U << 2);
inline constexpr uint8_t opa_count = 2;

inline constexpr bool has_opa(uint8_t n) {
    return n < 16U && (opa_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// One advanced-control timer (TIM1) and three general-purpose ones
/// (TIM2..4); the 32-bit TIM5 is the CH32V203RB's alone.
inline constexpr uint8_t advanced_timer_count = 1;
inline constexpr uint8_t general_timer_count  = 3;
inline constexpr bool has_tim5 = false;
/// The timers as masks, one bit per instance number: TIM1 the advanced
/// one, TIM2..TIM4 the general-purpose ones, and no basic timer
/// (chapter 16's TIM6 and TIM7 are the CH32V303RC's and VC's).
inline constexpr uint16_t advanced_timer_instances = (1U << 1);
inline constexpr uint16_t general_timer_instances  = (1U << 2) | (1U << 3) | (1U << 4);
inline constexpr uint16_t basic_timer_instances    = 0;
/// The CH32V303's blocks, none of which this series carries (datasheet
/// table 2-1): the DAC, the RNG, the SDIO host and the FSMC.
inline constexpr bool has_dac  = false;
inline constexpr bool has_rng  = false;
inline constexpr bool has_sdio = false;
inline constexpr bool has_fsmc = false;
inline constexpr bool has_ethernet = false;

/// Both USB blocks reach a pad here: the full-speed DEVICE controller
/// (USBD, RM ch. 21) on PA11/PA12, and the host/device one (USBFS, ch.
/// 23) on PB6/PB7. Which of them a BOARD wires to a connector is the
/// board's business, not the part's.
inline constexpr bool has_usbd  = true;
inline constexpr bool has_usbfs = true;

/// The EXTI lines this part has, one bit per line (RM table 9-3 and its
/// class notes): the sixteen pin lines, the PVD's (16) and the RTC
/// alarm's (17) on every part, and above them only what a block of this
/// part is wired to - here lines 0..18 and 20: the USB device
/// controller's wake-up on 18 and the USBFS controller's on 20.
inline constexpr uint32_t exti_lines = 0x17FFFFUL;

// ---- the clock tree's edges (datasheet 2.1 and tables 4-9, 4-11) ----------
/// The ceiling this part is rated for, whether its package brings out
/// the oscillator pads at all, and the crystal range its HSE takes.
inline constexpr uint32_t sysclk_max_hz = 144'000'000UL;
inline constexpr bool has_hse_pins      = true;
inline constexpr uint32_t hse_min_hz    = 3'000'000UL;
inline constexpr uint32_t hse_max_hz    = 25'000'000UL;

/// The PLL's own edges (datasheet table 4-15): what its input may be,
/// and what its output may be. Both are checked at compile time, and
/// they are NOT the same numbers on every part of the series.
inline constexpr uint32_t pll_in_min_hz  = 3'000'000UL;
inline constexpr uint32_t pll_in_max_hz  = 25'000'000UL;
inline constexpr uint32_t pll_out_min_hz = 18'000'000UL;
inline constexpr uint32_t pll_out_max_hz = 144'000'000UL;

/// What PLLXTPRE selects on the way from the HSE into the PLL, by code
/// (RM 3.4.2): the whole clock or half of it on this class, where the
/// CH32V203RB divides its 32 MHz oscillator by four or by eight.
inline constexpr uint8_t pll_hse_div[2] = {1, 2};

/// Whether USBPRE's fourth code (the PLL divided by five, from a PLL at
/// 240 MHz) exists here: it is the CH32V20x_D8's and the D8W's, and
/// even there it depends on the lot number (RM 3.4.2).
inline constexpr bool has_usb_pre_div5 = false;

/// The LSI as the datasheet measures it (table 4-14), not as the clock
/// tree's block diagram rounds it: a wide RC, which is why anything
/// timed by it is measured rather than computed.
inline constexpr uint32_t lsi_min_hz = 25'000UL;
inline constexpr uint32_t lsi_typ_hz = 39'000UL;
inline constexpr uint32_t lsi_max_hz = 60'000UL;

} // namespace brio::device
