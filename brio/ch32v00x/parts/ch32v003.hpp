/*
 * parts/ch32v003.hpp
 *
 * The CH32V003 (QingKe V2A) as the stratum's device tables know it:
 * the facts that tell this part from the CH32V006, stated once and
 * read by every driver through `device::`. Included by device.hpp
 * alone, under the build's part definition (CH32V003). Documents of
 * record: the CH32V003 datasheet V1.6 (1.4.2 for the memories, 2.2 for
 * the pins) and the part's register description as the family manual
 * states it for the CH32V006 where the two agree (docs/ch32v00x/
 * README.md lists what is tiered and what is still closed on this
 * part).
 *
 * The same reserve rule as parts/ch32v006.hpp: values here, `if
 * constexpr` in the drivers, the two macros for per-instance code.
 */

#pragma once

#include <stdint.h>

#define BRIO_CH32_HAS_TIM3 0
#define BRIO_CH32_HAS_USART2 0
#define BRIO_CH32_HAS_FLASH_HEAP 0
#define BRIO_CH32_PART_V003 1
#define BRIO_CH32_PART_V006 0

namespace brio {

enum class Ch32Part : uint8_t { v003, v006 };

namespace device {

inline constexpr Ch32Part part = Ch32Part::v003;
inline constexpr const char* part_name = "CH32V003";

/// Memory (DS 1.4.2): 16 KB of code flash, the 1920-byte boot loader
/// in its own system area; 2 KB of SRAM.
inline constexpr uint32_t flash_bytes = 16u * 1024u;
inline constexpr uint32_t sram_bytes = 2u * 1024u;
/// FLASH: the fast page, program AND erase unit, is 64 bytes on this
/// part (DS table 3-14), and the standard halfword program exists.
inline constexpr uint32_t flash_page_bytes = 64;
inline constexpr bool flash_has_halfword_program = true;
inline constexpr bool flash_has_block_erase = false;
/// 0 waits to 24 MHz, 1 wait to 48 MHz.
constexpr uint32_t flash_latency_for(uint32_t hz) { return hz <= 24'000'000UL ? 0UL : 1UL; }
/// The storage partition: the linker's 15 KB and a 1 KB journal attic
/// of sixteen 64-byte pages; no heap - a 16 KB array has no room for
/// a block allocator.
inline constexpr uint32_t flash_program_bytes = 15u * 1024u;
inline constexpr uint32_t flash_journal_pages = 16;
inline constexpr bool has_flash_heap = false;

/// The instances and the bonding: ports A (PA1, PA2), C and D; one
/// timer pair, one USART, the OPA as three bits of EXTEN_CTR.
inline constexpr bool has_port_b = false;
inline constexpr bool has_tim3 = false;
inline constexpr bool has_usart2 = false;
inline constexpr bool has_opa_block = false;

/// GPIO: MODE is TWO bits, the F1's - 00 input, 01 output at 10 MHz,
/// 10 at 2 MHz, 11 at 30 MHz. The stratum drives its outputs at the
/// top speed, which is the CH32V006's one speed.
inline constexpr uint32_t gpio_mode_output = 0x3u;

/// AFIO (RM 7.3.2, table 7-16): a reserved word, then PCFR1 at 0x04
/// and EXTICR at 0x08.
inline constexpr uint32_t afio_pcfr1_offset = 0x04u;
inline constexpr uint32_t afio_exticr_offset = 0x08u;

/// RCC: no system clock monitor on this part (CTLR bits 23:20 are
/// reserved); the HSE and its clock security system as on the CH32V006.
inline constexpr bool has_clock_monitor = false;
inline constexpr uint32_t hse_min_hz = 4'000'000UL;
inline constexpr uint32_t hse_max_hz = 25'000'000UL;

/// ADC (RM ch. 9, DS 3.3.14): a 10-bit converter; a conversion is the
/// sample time plus 11 ADCCLK cycles, the sample-time table in whole
/// cycles; the F1's calibration (RSTCAL then CAL) the CH32V006 has
/// not; the Vcal source on channel 9 at 2/4 or 3/4 AVDD (CALVOL) where
/// the CH32V006 has the OPA; the trigger delay register DLYR at 0x50
/// where the CH32V006 has CTLR3 - so no low-power mode, no input
/// buffer, no watchdogs 1 and 2 and no watchdog reset; fADC 1..12 MHz.
/// The OPA's output reaches the converter through PD4, channel 7.
inline constexpr uint32_t adc_bits = 10;
inline constexpr bool adc_has_ctlr3 = false;
inline constexpr bool adc_has_calibration = true;
inline constexpr bool adc_has_trigger_delay = true;
inline constexpr uint32_t adc_max_clock_hz = 12'000'000UL;
inline constexpr uint8_t opa_adc_channel = 7;

/// PWR (RM ch. 2): PLS is three bits (2.85 .. 4.4 V rising, the
/// 2.7 .. 5.5 V supply's monitor); no LDO_MODE and no FLASH_LP - the
/// regulator's and the flash's low-power settings are the CH32V006's.
/// The AWU's window and prescaler tables are the same on both parts.
inline constexpr bool pwr_has_ldo_modes = false;
inline constexpr bool pwr_has_flash_low_power = false;
inline constexpr uint32_t pvd_level_bits = 3;

/// TIM2 has no dead-time generator (RM 11.2: "lacks dead-time
/// generation"; its register list ends at CH4CVR).
inline constexpr bool tim2_has_dead_time = false;

/// USART: the F1's full register description - the synchronous mode
/// (CLKEN, CPOL, CPHA, LBCL) and the smartcard (SCEN, NACK, the guard
/// time) exist on this part; their verbs arrive with the tier that
/// measures them.
inline constexpr bool usart_has_synchronous = true;
inline constexpr bool usart_has_smartcard = true;

/// The vector table's length: entries 0..38 (TIM2 last).
inline constexpr uint8_t vector_count = 39;

} // namespace device
} // namespace brio
