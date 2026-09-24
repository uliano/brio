/*
 * parts/ch32v203rb.hpp
 *
 * The CH32V203RB as a table of facts: 128 KB of flash, 64 KB of SRAM,
 * LQFP64M with 49 bonded pins, and WCH's CH32V20x_D8 device class - the
 * one part of this series that is not D6. The rules this file obeys are
 * parts/ch32v203c8.hpp's; read that one first.
 *
 * THE OTHER DEVICE CLASS, AND WHAT FOLLOWS FROM IT. The reference
 * manual covers four families at once and keys many of its register
 * descriptions to a class; every other part of this series is
 * CH32V20x_D6 and this one is CH32V20x_D8 (RM, "Specific
 * classification abbreviations"), which is the classification the
 * manual derives from flash size alone - D6 is 32 or 64 KB, D8 is 128
 * or 256. Three things follow that no other part here has: a 32-bit
 * general-purpose timer (TIM5), a 10M Ethernet with its own PHY, and a
 * VECTOR TABLE WITH A DIFFERENT TAIL - seventy entries against the D6's
 * sixty-three, with the Ethernet pair, TIM5, UART4, the eighth DMA
 * channel and the 32 kHz oscillator's two lines after the USBFS pair.
 * The manual's own table 9-2 is the union of this family with the
 * CH32V30x and states neither tail, so the crt states the per-class
 * truth from the part definition (startup_ch32v203.S) and
 * device.hpp's Irq table carries it line by line.
 *
 * ITS CRYSTAL IS 32 MHz AND NOTHING ELSE (datasheet tables 4-9 and
 * 4-11, both with "applied for V203RBT6" beside the figure), where
 * every other part takes 3 to 25 - and it needs no external load
 * capacitors, which are built in.
 *
 * THE MEMORY SPLIT IS CONFIGURABLE. The datasheet's note 2 offers three
 * divisions of the same array - 128K flash with 64K SRAM, 144K with
 * 48K, 160K with 32K - chosen by a word in the configuration memory.
 * The first is what this file and ld/ch32v203rb.ld state, because it is
 * the one a part leaves the factory in; a program that moves the line
 * moves both.
 */

#pragma once

#include <stdint.h>

namespace brio::device {

/// The name this part answers to on a console banner.
inline constexpr const char* part_name = "CH32V203RB";

// ---- the memories (datasheet table 2-1 and its note 2) --------------------
inline constexpr uint32_t flash_bytes = 128UL * 1024UL;
inline constexpr uint32_t sram_bytes  = 64UL * 1024UL;
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
/// This part's window is the factory split of note 2 (128K of flash
/// with 64K of SRAM); the option byte that moves it moves the tail with
/// it.
inline constexpr uint32_t flash_tail_bytes  = 96UL * 1024UL;
inline constexpr uint32_t flash_array_bytes = 224UL * 1024UL;

// ---- the device class -----------------------------------------------------
/// WCH's own family division, which several chapters are keyed by: this
/// part is CH32V20x_D8, the only one of this series (RM, "Specific
/// classification abbreviations"). A register description that names
/// another class is about other silicon; a driver asks the facts below
/// and names the class only where the manual's note is a list of classes.
inline constexpr DeviceClass device_class = DeviceClass::v20x_d8;
/// The core: the QingKe V4B, RV32IMAC with no floating-point unit.
inline constexpr bool has_fpu = false;
/// The vector table's length, which follows the class: 70 entries, the
/// last being the 32 kHz oscillator's wake-up (startup_ch32v203.S).
inline constexpr uint32_t vector_count = 70;

/// The backup domain's DATA registers, which the DEVICE CLASS decides:
/// forty-two here - BKP_DATAR1..BKP_DATAR10 and BKP_DATAR11..BKP_DATAR42
/// in a second block above the tamper registers, eighty-four bytes -
/// where the CH32V20x_D6 carries ten (RM 4.3's note under table 4-1).
inline constexpr uint8_t bkp_data_registers = 42;

/// What RTCSEL's third choice divides the HSE by on the way to the RTC
/// (RM 3.4.9). The register description names this device class among
/// those that divide by 512 and says nothing about the lot, so the two
/// entries are equal - the CH32V20x_D6's are not.
inline constexpr uint32_t rtc_hse_div[2] = {512, 512};

// ---- the DMA controllers (RM 11.3's register notes) -----------------------
/// One controller here, with EIGHT channels: RM 11.3.1's notes give the
/// eighth to this device class among the five that have it.
inline constexpr uint8_t dma_controller_count = 1;
inline constexpr uint8_t dma1_channel_count   = 8;
inline constexpr uint8_t dma2_channel_count   = 0;

// ---- the pads (datasheet 3.2, table 3-1-4) --------------------------------
/// Which pins each port bonds, as a mask. Three whole ports and one pin
/// of port D: PD2 is pin 54, and PD0/PD1 do NOT exist on this part -
/// where every other package can hand its two oscillator pads back to
/// port D, "CH32V203RBT6 chip only has OSC_IN and OSC_OUT function
/// pins, which cannot be reused as PD0 and PD1" (the datasheet's note
/// 4). Port E is not bonded on any part of this series.
inline constexpr uint16_t port_pins(char port) {
    return port == 'A' ? 0xFFFFU :
           port == 'B' ? 0xFFFFU :
           port == 'C' ? 0xFFFFU :
           port == 'D' ? 0x0004U :   // PD2
           0x0000U;
}

inline constexpr bool has_port(char port) { return port_pins(port) != 0U; }

/// The two pads the debug port owns from reset: a program that takes
/// them for itself loses the probe until the next power cycle.
inline constexpr char debug_swdio_port = 'A';
inline constexpr uint8_t debug_swdio_pin = 13;
inline constexpr char debug_swclk_port = 'A';
inline constexpr uint8_t debug_swclk_pin = 14;

// ---- the instances (datasheet table 2-1) ----------------------------------
/// The USARTs this part offers, one bit per instance number: all four,
/// and this package bonds the default pads of every one of them.
inline constexpr uint16_t usart_instances = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4);
inline constexpr uint8_t usart_count = 4;

inline constexpr bool has_usart(uint8_t n) {
    return n < 16U && (usart_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// Which of those instances are FULL USARTs - the synchronous clock on
/// a CK pad, the smartcard and the flow-control pair - rather than
/// asynchronous receivers alone. THE FOURTH IS NOT ONE HERE: chapter
/// 18's opening gives this class a UART4, the remap table that starts
/// at PC10/PC11 carries TX and RX and nothing else, and the datasheet's
/// block diagram names "UART4 RX, TX" beside the three USARTs' five
/// signals.
inline constexpr uint16_t usart_full_instances =
    static_cast<uint16_t>(usart_instances & ~static_cast<uint16_t>(1U << 4));

inline constexpr bool usart_full(uint8_t n) {
    return n < 16U && (usart_full_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

inline constexpr uint8_t spi_count = 2;
/// The synchronous ports as a mask, one bit per instance number: SPI1
/// and SPI2 here. The I2S face of SPI2 and SPI3 is the CH32V303RC's and
/// VC's; this series has none (datasheet table 2-1 has no I2S row).
inline constexpr uint16_t spi_instances = (1U << 1) | (1U << 2);
inline constexpr bool has_i2s = false;
inline constexpr uint8_t i2c_count = 2;
inline constexpr uint8_t can_count = 1;
/// ONE converter, not the two every other part of the series carries,
/// with all sixteen analog channels on a pad: ADC_IN0..7 are PA0..PA7,
/// 8 and 9 are PB0 and PB1, 10..15 are PC0..PC5 (datasheet 3.3, and
/// table 2-1's "16@1" against the others' "10@2").
inline constexpr uint8_t adc_count         = 1;
inline constexpr uint8_t adc_channel_count = 16;

/// The operational amplifiers, one bit per instance number: both, each
/// with an input pair and an output on a bonded pad.
inline constexpr uint16_t opa_instances = (1U << 1) | (1U << 2);
inline constexpr uint8_t opa_count = 2;

inline constexpr bool has_opa(uint8_t n) {
    return n < 16U && (opa_instances & static_cast<uint16_t>(1U << n)) != 0U;
}

/// One advanced-control timer (TIM1), three general-purpose 16-bit ones
/// (TIM2..4) and - this part alone - the 32-bit TIM5.
inline constexpr uint8_t advanced_timer_count = 1;
inline constexpr uint8_t general_timer_count  = 3;
inline constexpr bool has_tim5 = true;
/// The timers as masks, one bit per instance number: TIM1 the advanced
/// one, TIM2..TIM4 and the 32-bit TIM5 the general-purpose ones, and no
/// basic timer (chapter 16's TIM6 and TIM7 are the CH32V303RC's and
/// VC's).
inline constexpr uint16_t advanced_timer_instances = (1U << 1);
inline constexpr uint16_t general_timer_instances  = (1U << 2) | (1U << 3) | (1U << 4) | (1U << 5);
inline constexpr uint16_t basic_timer_instances    = 0;
/// The CH32V303's blocks, none of which this series carries (datasheet
/// table 2-1): the DAC, the RNG, the SDIO host and the FSMC.
inline constexpr bool has_dac  = false;
inline constexpr bool has_rng  = false;
inline constexpr bool has_sdio = false;
inline constexpr bool has_fsmc = false;
inline constexpr bool has_ethernet = true;

/// Both USB blocks reach a pad here: the full-speed DEVICE controller
/// (USBD, RM ch. 21) on PA11/PA12, and the host/device one (USBFS, ch.
/// 23) on PB6/PB7.
inline constexpr bool has_usbd  = true;
inline constexpr bool has_usbfs = true;

// ---- the clock tree's edges (datasheet 2.1 and tables 4-9, 4-11) ----------
/// The ceiling this part is rated for, that its package brings out the
/// oscillator pads, and the ONE crystal rate its HSE takes - 32 MHz,
/// stated as a range of one so that every arithmetic written against
/// the family keeps working (the file header).
inline constexpr uint32_t sysclk_max_hz = 144'000'000UL;
inline constexpr bool has_hse_pins      = true;
inline constexpr uint32_t hse_min_hz    = 32'000'000UL;
inline constexpr uint32_t hse_max_hz    = 32'000'000UL;

/// The PLL's own edges (datasheet table 4-15): this part's input floor
/// is 4 MHz and its output reaches 240 MHz - the rate the USB
/// prescaler's fourth code divides by five - where every other part of
/// the series stops at 144.
inline constexpr uint32_t pll_in_min_hz  = 4'000'000UL;
inline constexpr uint32_t pll_in_max_hz  = 25'000'000UL;
inline constexpr uint32_t pll_out_min_hz = 40'000'000UL;
inline constexpr uint32_t pll_out_max_hz = 240'000'000UL;

/// What PLLXTPRE selects on the way from the HSE into the PLL, by code
/// (RM 3.4.2): on this class the 32 MHz oscillator arrives divided by
/// four or by eight - never whole - so the PLL's input is 8 or 4 MHz.
inline constexpr uint8_t pll_hse_div[2] = {4, 8};

/// Whether USBPRE's fourth code (the PLL divided by five, from a PLL at
/// 240 MHz) exists here: on this class it does, for lot numbers whose
/// penultimate digit is greater than zero (RM 3.4.2), which is a fact
/// about the die in hand and not about the part number.
inline constexpr bool has_usb_pre_div5 = true;

/// The LSI as the datasheet measures it (table 4-14): this part's is
/// the 32 kHz one, where the rest of the series carries the 40 kHz RC.
inline constexpr uint32_t lsi_min_hz = 25'000UL;
inline constexpr uint32_t lsi_typ_hz = 32'000UL;
inline constexpr uint32_t lsi_max_hz = 45'000UL;

} // namespace brio::device
