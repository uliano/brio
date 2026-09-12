/*
 * device.hpp
 *
 * The RP2040's device description, in one include: the CMSIS header
 * generated from the chip's SVD (the register blocks as structs, the
 * instance pointers, the IRQn enumerators, the core's revision and
 * priority width, core_cm0plus.h behind it) and the pico-sdk's
 * bit-field definitions for every register of every block - both from
 * third_party/pico-sdk/, the SVD's two halves. Every header of this
 * stratum includes this one first; the `armv6m/` core files are
 * written against the CMSIS half and refuse to be included before it.
 *
 * WHY TWO HALVES. The CMSIS header names the registers (UART0->UARTFR)
 * and nothing inside them; the regs headers name every field
 * (UART_UARTFR_TXFF_BITS) and no struct. The SDK's own structs
 * (hardware/structs/) are not taken: they would be a third spelling of
 * the same map. hardware/regs/addressmap.h is NOT included: its *_BASE
 * macros are the CMSIS header's too, and -Werror would see the
 * redefinition.
 *
 * ATOMIC REGISTER ALIASES (2.1.2). Every peripheral register on the
 * APB and AHB-Lite fabric answers at three more addresses: +0x1000
 * XORs the written bits in, +0x2000 sets them, +0x3000 clears them -
 * one bus write, no read-modify-write, so a bit can be flipped from
 * an interrupt handler or from the OTHER CORE without a critical
 * section. hw_set / hw_clear / hw_xor below are those three verbs over
 * any register the CMSIS structs name; the SIO block has its own
 * SET/CLR/XOR registers instead and never needs them.
 */

#pragma once

#include <stdint.h>

#include "RP2040.h"

#include "hardware/platform_defs.h"
#include "hardware/regs/adc.h"
#include "hardware/regs/busctrl.h"
#include "hardware/regs/clocks.h"
#include "hardware/regs/dma.h"
#include "hardware/regs/dreq.h"
#include "hardware/regs/i2c.h"
#include "hardware/regs/intctrl.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/io_qspi.h"
#include "hardware/regs/m0plus.h"
#include "hardware/regs/pads_bank0.h"
#include "hardware/regs/pads_qspi.h"
#include "hardware/regs/pio.h"
#include "hardware/regs/pll.h"
#include "hardware/regs/psm.h"
#include "hardware/regs/pwm.h"
#include "hardware/regs/resets.h"
#include "hardware/regs/rosc.h"
#include "hardware/regs/rtc.h"
#include "hardware/regs/sio.h"
#include "hardware/regs/spi.h"
#include "hardware/regs/ssi.h"
#include "hardware/regs/syscfg.h"
#include "hardware/regs/sysinfo.h"
#include "hardware/regs/tbman.h"
#include "hardware/regs/timer.h"
#include "hardware/regs/uart.h"
#include "hardware/regs/usb.h"
#include "hardware/regs/usb_device_dpram.h"
#include "hardware/regs/vreg_and_chip_reset.h"
#include "hardware/regs/watchdog.h"
#include "hardware/regs/xip.h"
#include "hardware/regs/xosc.h"

namespace brio {

/// The three write-only aliases of 2.1.2, as offsets from a register's
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
/// spell their thirty per-pin registers as thirty struct members, and a
/// verb indexed by pin number wants the address arithmetic the SVD's
/// offsets describe instead.
inline volatile uint32_t& reg_at(uint32_t base, uint32_t offset) {
    return *reinterpret_cast<volatile uint32_t*>(static_cast<uintptr_t>(base + offset));
}

} // namespace brio
