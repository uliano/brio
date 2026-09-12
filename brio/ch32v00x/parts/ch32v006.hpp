/*
 * parts/ch32v006.hpp
 *
 * The CH32V006 (QingKe V2C) as the stratum's device tables know it:
 * the facts that tell this part from the CH32V003, stated once and
 * read by every driver through `device::`. Included by device.hpp
 * alone, under the build's part definition (CH32V006).
 *
 * THIS IS THE RESERVE OF THIS FAMILY (docs/design/overview.md,
 * "Generalization rule"): with no vendor header to probe, the part is
 * a build definition, asked in device.hpp once, and everything a
 * driver needs to know about the part is a constexpr value here - a
 * driver branches with `if constexpr` on these and never asks the
 * preprocessor which part it is on. The two macros below are the
 * exception the rule allows: per-INSTANCE code (a register struct, a
 * vector table entry) that no value can compile out.
 */

#pragma once

#include <stdint.h>

#define BRIO_CH32_HAS_TIM3 1
#define BRIO_CH32_HAS_USART2 1
#define BRIO_CH32_HAS_FLASH_HEAP 1
#define BRIO_CH32_PART_V003 0
#define BRIO_CH32_PART_V006 1

namespace brio {

enum class Ch32Part : uint8_t { v003, v006 };

namespace device {

inline constexpr Ch32Part part = Ch32Part::v006;
inline constexpr const char* part_name = "CH32V006";

/// Memory (DS table 1-1): 62 KB of the array to the program and the
/// storage classes, the top 2 KB the boot loader's; 8 KB of SRAM.
inline constexpr uint32_t flash_bytes = 62u * 1024u;
inline constexpr uint32_t sram_bytes = 8u * 1024u;
/// FLASH (RM ch. 18): the fast page, program AND erase unit.
inline constexpr uint32_t flash_page_bytes = 256;
inline constexpr bool flash_has_halfword_program = false;   ///< no PG: the page is the only write
inline constexpr bool flash_has_block_erase = true;         ///< BER32
/// RM 18.3.1: 0 waits to 15 MHz, 1 wait to 24 MHz, 2 waits to 48 MHz.
constexpr uint32_t flash_latency_for(uint32_t hz) {
    return hz <= 15'000'000UL ? 0UL : hz <= 24'000'000UL ? 1UL : 2UL;
}
/// The storage partition (nvm_flash.hpp): the linker's 40 KB, the
/// heap's 16 KB, the journal's 6 KB attic - as page counts here.
inline constexpr uint32_t flash_program_bytes = 40u * 1024u;
inline constexpr uint32_t flash_journal_pages = 24;
inline constexpr bool has_flash_heap = true;

/// The instances and the bonding this part has that the CH32V003 has not.
inline constexpr bool has_port_b = true;
inline constexpr bool has_tim3 = true;
inline constexpr bool has_usart2 = true;
inline constexpr bool has_opa_block = true;   ///< the key-locked OPA/CMP block (RM ch. 17)

/// GPIO (RM 7.3.1.1): MODE is ONE bit - input, or output at the port's
/// only speed - and the nibble's second MODE bit is reserved.
inline constexpr uint32_t gpio_mode_output = 0x1u;

/// AFIO: PCFR1 and EXTICR in the block's register space.
inline constexpr uint32_t afio_pcfr1_offset = 0x0Cu;
inline constexpr uint32_t afio_exticr_offset = 0x08u;

/// RCC: the system clock monitor (SYSCM_EN), the HSE and its clock
/// security system.
inline constexpr bool has_clock_monitor = true;
inline constexpr uint32_t hse_min_hz = 4'000'000UL;
inline constexpr uint32_t hse_max_hz = 25'000'000UL;

/// ADC (RM ch. 9, DS 3.3.x): a 12-bit converter; a conversion is the
/// sample time plus 12.5 ADCCLK cycles, the sample-time table in half
/// cycles with two columns (CTLR3.ADC_LP); no calibration; the OPA's
/// output on channel 9, an internal route; CTLR3 at 0x50 (the low-power
/// mode, the input buffer, watchdogs 1 and 2, the watchdog reset) and
/// no trigger delay register; fADC to 48 MHz.
inline constexpr uint32_t adc_bits = 12;
inline constexpr bool adc_has_ctlr3 = true;
inline constexpr bool adc_has_calibration = false;
inline constexpr bool adc_has_trigger_delay = false;
inline constexpr uint32_t adc_max_clock_hz = 48'000'000UL;
inline constexpr uint8_t opa_adc_channel = 9;

/// PWR (RM ch. 2): PLS is two bits (1.87 .. 2.66 V rising), and the
/// regulator (LDO_MODE) and the flash (FLASH_LP) have low-power
/// settings of their own.
inline constexpr bool pwr_has_ldo_modes = true;
inline constexpr bool pwr_has_flash_low_power = true;
inline constexpr uint32_t pvd_level_bits = 2;

/// TIM2 pairs its channels with a dead time through DTCR (RM 12.4.17).
inline constexpr bool tim2_has_dead_time = true;

/// USART (RM ch. 14): the register description has neither the
/// synchronous mode nor the smartcard (measured: the bits do not
/// stick).
inline constexpr bool usart_has_synchronous = false;
inline constexpr bool usart_has_smartcard = false;

/// The vector table's length: entries 0..40 (USART2 at 39, OPCM at 40).
inline constexpr uint8_t vector_count = 41;

} // namespace device
} // namespace brio
