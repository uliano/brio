/*
 * device.hpp
 *
 * The RP2350's device description, in one include: the CMSIS header
 * generated from the chip's SVD (the register blocks as structs, the
 * instance pointers, the IRQn enumerators, the core's revision and
 * priority width) and the pico-sdk's bit-field definitions for every
 * register of every block - both from third_party/pico-sdk/rp2350/, the
 * SVD's two halves. Every header of this stratum includes this one
 * first.
 *
 * AND IT IS THE SAME FILE ON BOTH ARCHITECTURES. This chip carries two
 * Cortex-M33 cores and two Hazard3 RISC-V cores over one set of
 * peripherals, and the IRQ NUMBERING IS SHARED (datasheet 3.8.4.2), so
 * `RP2350.h`'s IRQn enumerators are as true of a hart as of an M33.
 * What is architecture-specific is the CORE, and RP2350.h's
 * unconditional `#include "core_cm33.h"` is the one thing the RISC-V
 * build must not take: its preset puts
 * third_party/pico-sdk/rp2350/no_core/ first on the include path, where
 * that name resolves to a stub. Nothing here, and nothing above here,
 * asks which processor is in the socket - brio/rp2350/core.hpp is the
 * ONE file that does.
 *
 * WHY TWO HALVES. The CMSIS header names the registers (UART0->UARTFR)
 * and nothing inside them; the regs headers name every field
 * (UART_UARTFR_TXFF_BITS) and no struct. The SDK's own structs
 * (hardware/structs/) are not taken: they would be a third spelling of
 * the same map. hardware/regs/addressmap.h is NOT included: its *_BASE
 * macros are the CMSIS header's too, and -Werror would see the
 * redefinition.
 *
 * ATOMIC REGISTER ALIASES (2.1.3). Every peripheral register on the APB
 * and AHB fabric answers at three more addresses: +0x1000 XORs the
 * written bits in, +0x2000 sets them, +0x3000 clears them - one bus
 * write, no read-modify-write, so a bit can be flipped from an interrupt
 * handler or from another core without a critical section. hw_set /
 * hw_clear / hw_xor below are those three verbs over any register the
 * CMSIS structs name; the SIO block has its own SET/CLR/XOR registers
 * instead and never needs them.
 *
 * THE PACKAGE IS A BUILD FACT AND A SILICON FACT AT ONCE. This chip
 * comes in two packages that differ in what is BONDED: QFN-60 brings out
 * 30 GPIO and four ADC inputs, QFN-80 brings out 48 and eight (1.2). The
 * die is one, so every register of every pin exists on both; what does
 * not exist is the pad. A build states the package it is for
 * (BRIO_RP2350_PACKAGE_PINS, rp2350/CMakeLists.txt) and then a pin the
 * package has not got is a COMPILE error; a build that states nothing
 * compiles for the larger package and the refusal happens at RUN time,
 * against SYSINFO.PACKAGE_SEL (rp2350/sysinfo.hpp). This file is where
 * both truths are stated once, and every later chapter reads them from
 * here rather than spelling 30 or 48 again.
 */

#pragma once

#include <stdint.h>

#include "RP2350.h"

#include "hardware/platform_defs.h"
#include "hardware/regs/accessctrl.h"
#include "hardware/regs/adc.h"
#include "hardware/regs/bootram.h"
#include "hardware/regs/busctrl.h"
#include "hardware/regs/clocks.h"
#include "hardware/regs/coresight_trace.h"
#include "hardware/regs/dma.h"
#include "hardware/regs/dreq.h"
#include "hardware/regs/glitch_detector.h"
#include "hardware/regs/hstx_ctrl.h"
#include "hardware/regs/hstx_fifo.h"
#include "hardware/regs/i2c.h"
#include "hardware/regs/intctrl.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/io_qspi.h"
#include "hardware/regs/m33.h"
#include "hardware/regs/m33_eppb.h"
#include "hardware/regs/otp.h"
#include "hardware/regs/otp_data.h"
#include "hardware/regs/pads_bank0.h"
#include "hardware/regs/pads_qspi.h"
#include "hardware/regs/pio.h"
#include "hardware/regs/pll.h"
#include "hardware/regs/powman.h"
#include "hardware/regs/psm.h"
#include "hardware/regs/pwm.h"
#include "hardware/regs/qmi.h"
#include "hardware/regs/resets.h"
#include "hardware/regs/riscv_dm.h"
#include "hardware/regs/rosc.h"
#include "hardware/regs/rp_ap.h"
#include "hardware/regs/rvcsr.h"
#include "hardware/regs/sha256.h"
#include "hardware/regs/sio.h"
#include "hardware/regs/spi.h"
#include "hardware/regs/syscfg.h"
#include "hardware/regs/sysinfo.h"
#include "hardware/regs/tbman.h"
#include "hardware/regs/ticks.h"
#include "hardware/regs/timer.h"
#include "hardware/regs/trng.h"
#include "hardware/regs/uart.h"
#include "hardware/regs/usb.h"
#include "hardware/regs/usb_device_dpram.h"
#include "hardware/regs/watchdog.h"
#include "hardware/regs/xip.h"
#include "hardware/regs/xip_aux.h"
#include "hardware/regs/xosc.h"

namespace brio {

/// The three write-only aliases of 2.1.3, as offsets from a register's
/// normal address.
inline constexpr uint32_t reg_alias_xor = 0x1000u;
inline constexpr uint32_t reg_alias_set = 0x2000u;
inline constexpr uint32_t reg_alias_clr = 0x3000u;

inline volatile uint32_t& reg_alias(volatile uint32_t& reg, uint32_t alias) {
    return *reinterpret_cast<volatile uint32_t*>(
        reinterpret_cast<uintptr_t>(&reg) + alias);
}

/// reg |= bits, in one bus write.
inline void hw_set(volatile uint32_t& reg, uint32_t bits) { reg_alias(reg, reg_alias_set) = bits; }
/// reg &= ~bits, in one bus write.
inline void hw_clear(volatile uint32_t& reg, uint32_t bits) { reg_alias(reg, reg_alias_clr) = bits; }
/// reg ^= bits, in one bus write.
inline void hw_xor(volatile uint32_t& reg, uint32_t bits) { reg_alias(reg, reg_alias_xor) = bits; }
/// (reg & ~mask) | (value & mask), as one XOR alias write: the bits that
/// differ are flipped, the rest untouched - still one bus write, but the
/// read before it is a read-modify-write again; fine for configuration,
/// not for a field two contexts race on.
inline void hw_write_masked(volatile uint32_t& reg, uint32_t value, uint32_t mask) {
    hw_xor(reg, (reg ^ value) & mask);
}

/// A register reached by address: the IO_BANK0 and PADS_BANK0 blocks
/// spell their per-pin registers as struct members, and a verb indexed by
/// pin number wants the address arithmetic the SVD's offsets describe
/// instead.
inline volatile uint32_t& reg_at(uint32_t base, uint32_t offset) {
    return *reinterpret_cast<volatile uint32_t*>(static_cast<uintptr_t>(base + offset));
}

// ---- the package, the one fact a build may state about the silicon ------

/// The two packages of this chip, by their pin counts (1.2).
enum class Package : uint8_t {
    qfn60 = 60,   ///< 30 GPIO, four ADC inputs (GP26..GP29)
    qfn80 = 80,   ///< 48 GPIO, eight ADC inputs (GP40..GP47)
};

/// What each package bonds. Stated here once; every chapter asks these.
constexpr uint8_t package_gpio_count(Package p) { return p == Package::qfn60 ? 30u : 48u; }
constexpr uint8_t package_adc_inputs(Package p) { return p == Package::qfn60 ? 4u : 8u; }
constexpr uint8_t package_adc_base_pin(Package p) { return p == Package::qfn60 ? 26u : 40u; }

#if defined(BRIO_RP2350_PACKAGE_PINS)
/// Whether the BUILD states which package this image is for: with it, a
/// pad the package has not got is a compile error; without it, the
/// refusal is a run-time one against SYSINFO.PACKAGE_SEL.
inline constexpr bool package_known = true;
static_assert(BRIO_RP2350_PACKAGE_PINS == 60 || BRIO_RP2350_PACKAGE_PINS == 80,
              "brio RP2350: BRIO_RP2350_PACKAGE_PINS is 60 (QFN-60) or 80 (QFN-80)");
inline constexpr Package package = static_cast<Package>(BRIO_RP2350_PACKAGE_PINS);
#else
inline constexpr bool package_known = false;
inline constexpr Package package = Package::qfn80;
#endif

/// The size of the user GPIO bank in THIS image: the package's when the
/// build states one, the larger package's otherwise.
inline constexpr uint8_t gpio_count = package_gpio_count(package);
/// The largest bank any package of this chip has - the size of the
/// register files, which exist on both because the die is one.
inline constexpr uint8_t gpio_count_max = package_gpio_count(Package::qfn80);

} // namespace brio
