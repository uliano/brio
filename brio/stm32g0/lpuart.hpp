/*
 * lpuart.hpp
 *
 * The LOW-POWER UART (RM0444 ch. 34), in this stratum's two strata:
 *
 *  Lpuart<n>              the RESOURCE - LPUART1 on every G0, LPUART2 on
 *                         the G0B1/G0C1 class;
 *  LpUart<n, pins, ...>   the TASK, which is stm32g0/usart.hpp's OWN
 *                         `UartTask` named over this resource. One
 *                         implementation, two peripherals.
 *
 * WHY THIS IS A SEPARATE HEADER AND NOT A TEMPLATE ARGUMENT OF THE
 * USART'S. Chapter 34 is chapter 33 with three differences, and only one
 * of them is small:
 *
 *  1. THE BAUD GENERATOR IS ANOTHER ARITHMETIC ALTOGETHER (34.4.7):
 *     baud = 256 x lpuart_ker_ck_pres / LPUARTDIV, LPUARTDIV a TWENTY-bit
 *     number in LPUART_BRR, values below 0x300 FORBIDDEN, and fck must
 *     sit between 3 x and 4096 x the baud rate. That is not "the USART's
 *     divisor with a factor": it is a fixed-point divisor with a
 *     legal window, and getting it wrong is a link that does not run.
 *  2. THE FEATURE SET IS THE `LP` COLUMN of table 184: no synchronous
 *     mode, no smartcard, no IrDA, no LIN, no auto-baud, no receiver
 *     time-out, no Modbus - and NO OVER8, because with 256 sub-steps
 *     there is nothing for it to do. What IS there, and is the point of
 *     the peripheral: the FIFOs, the prescaler, the four kernel clocks
 *     and the wake from Stop 0/1 on LSE or HSI16 - a serial port that
 *     keeps working while the rest of the chip is stopped.
 *  3. Every LPUART has a kernel-clock multiplexer (RCC_CCIPR's
 *     LPUART1SEL at bit 10 and LPUART2SEL at bit 8), where among the
 *     USARTs only the FULL ones do.
 *
 * Everything else - the register layout (the device header gives both
 * peripherals a USART_TypeDef), the enable rule, the flags in both
 * views, the ICR twins, the ORE storm, the FIFO's per-entry error flags,
 * single-wire half-duplex, mute mode, character match, driver enable and
 * flow control - is the USART's, and is REUSED rather than copied: this
 * file includes stm32g0/usart.hpp for the vocabulary and the task, and
 * the resource below answers the same questions the other one does.
 *
 * THE VECTOR IS SHARED (device_tables.hpp): on the G0B1 class LPUART1
 * arrives on USART3_4_5_6_LPUART1_IRQn and LPUART2 on
 * USART2_LPUART2_IRQn - the console's own line - so an application that
 * puts a console on USART2 and a second link on LPUART2 writes ONE
 * handler and calls both isr() bodies from it.
 *
 * THE WAKE LINES are EXTI 28 (LPUART1) and 35 (LPUART2), both DIRECT
 * (table 65). Line 35 lives in the EXTI's SECOND register group, which
 * only the G0B1 class has - and which stm32g0/exti.hpp already handles.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/usart.hpp"

namespace brio {

// ---- baud arithmetic (34.4.7) --------------------------------------------------

/// The lowest and highest LPUARTDIV the chapter allows. The floor is
/// stated ("it is forbidden to write values lower than 0x300"); the
/// ceiling is the field's own twenty bits.
constexpr uint32_t lpuart_div_min = 0x300u;
constexpr uint32_t lpuart_div_max = 0x000F'FFFFu;

/// LPUARTDIV for `baud` at kernel clock `hz` (already through the
/// prescaler): LPUARTDIV = 256 x hz / baud, rounded to nearest.
///
/// Nothing when the result leaves the legal window, which is the SAME
/// statement as 34.4.7's other rule - fck must be between 3 x and
/// 4096 x the baud rate - because 256 x 3 = 768 = 0x300 and
/// 256 x 4096 = 0x100000, one past the field. So the two constraints of
/// that section are ONE constraint, and this function is it.
constexpr std::optional<uint32_t> lpuart_brr(uint32_t hz, uint32_t baud) {
    if (hz == 0u || baud == 0u) {
        return std::nullopt;
    }
    // 256 x hz overflows 32 bits above 16.7 MHz, so the multiplication
    // is done in 64 bits - once, at compile time for a static clock.
    const uint64_t num = (static_cast<uint64_t>(hz) << 8) + (baud / 2u);
    const uint64_t div = num / baud;
    if (div < lpuart_div_min || div > lpuart_div_max) {
        return std::nullopt;
    }
    return static_cast<uint32_t>(div);
}

/// What the generator really produces for an LPUARTDIV value.
constexpr uint32_t lpuart_actual_baud(uint32_t hz, uint32_t brr) {
    if (brr < lpuart_div_min) {
        return 0;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(hz) << 8) / brr);
}

/// The lowest kernel clock that still reaches `baud`: 3 x it (34.4.7).
constexpr uint32_t lpuart_min_hz(uint32_t baud) { return baud * 3u; }
/// And the highest: 4096 x it.
constexpr uint32_t lpuart_max_hz(uint32_t baud) { return baud * 4096u; }

// ---- the resource -------------------------------------------------------------

/**
 * Lpuart<n>: the instance. The same surface Usart<n> offers - the task
 * above both of them asks nothing else - with this chapter's answers.
 */
template <uint8_t n>
struct Lpuart {
    static_assert(lpuart_present(n),
                  "brio Lpuart: this device has no such LPUART instance (LPUART1 is on "
                  "every STM32G0; LPUART2 is the G0B1/G0C1 class's alone)");

    Lpuart() = delete;

    static constexpr uint8_t index = n;
    /// EVERY LPUART has one (5.4.21), unlike the USARTs.
    static constexpr bool has_clock_select = lpuart_clock_select_pos(n) != 0xFF;

    /// Table 184's `LP` column, as the flags the shared task asks for.
    static constexpr bool is_full = false;        ///< it is LP, not FULL
    static constexpr bool has_prescaler = true;   ///< PRESC exists here
    static constexpr bool has_fifo_mode = true;   ///< and so do the 8-deep FIFOs
    /// The one row of table 184 the FULL/BASIC reading gets wrong the
    /// other way: synchronous mode is a USART's, FULL or BASIC, and NO
    /// LPUART has a CK output at all.
    static constexpr bool has_synchronous_mode = false;
    static constexpr bool has_receiver_timeout = false;
    static constexpr bool has_lin_mode = false;
    static constexpr bool has_oversampling8 = false;
    static constexpr uint8_t fifo_depth = 8;
    static constexpr uint8_t exti_line = lpuart_exti_line(n);
    static constexpr bool is_lpuart = true;

    static USART_TypeDef& regs() { return *reinterpret_cast<USART_TypeDef*>(lpuart_base(n)); }
    static constexpr IRQn_Type irq() { return lpuart_irq(n); }

    /// The header's own answers, at run time - the second opinion the
    /// bench compares the table above against (test_stm32_serial's
    /// letter a asks both, and the silicon third).
    static bool has_fifo() { return IS_UART_FIFO_INSTANCE(&regs()) != 0; }
    static bool has_half_duplex() { return IS_UART_HALFDUPLEX_INSTANCE(&regs()) != 0; }
    static bool has_flow_control() { return IS_UART_HWFLOW_INSTANCE(&regs()) != 0; }
    static bool has_driver_enable() { return IS_UART_DRIVER_ENABLE_INSTANCE(&regs()) != 0; }
    static bool has_wake_from_stop() {
        return IS_UART_WAKEUP_FROMSTOP_INSTANCE(&regs()) != 0;
    }
    /// And the four the LP column has NOT got, so a caller reading the
    /// resource sees the whole answer in one place.
    static bool has_autobaud() { return false; }
    static bool has_lin() { return false; }
    static bool has_synchronous() { return IS_USART_INSTANCE(&regs()) != 0; }
    static bool has_irda() { return false; }
    static bool has_smartcard() { return false; }

    /// BOTH LPUARTs sit on APBENR1 (LPUART1 at bit 20, LPUART2 at bit 7).
    static void bus_clock(bool on) {
        constexpr UsartBusClock bc = lpuart_bus_clock(n);
        if constexpr (bc.apb2) {
            Rcc::apb2_clock(bc.mask, on);
        } else {
            Rcc::apb1_clock(bc.mask, on);
        }
    }

    static bool kernel_clock(UsartClock c) {
        Rcc::kernel_clock(lpuart_clock_select_pos(n), static_cast<uint8_t>(c));
        return true;
    }
    static UsartClock kernel_clock() {
        return static_cast<UsartClock>(Rcc::kernel_clock(lpuart_clock_select_pos(n)));
    }

    static bool enabled() { return (regs().CR1 & USART_CR1_UE) != 0u; }
    static void enable(bool on) {
        regs().CR1 = on ? (regs().CR1 | USART_CR1_UE) : (regs().CR1 & ~USART_CR1_UE);
    }

    /// The frame and the divisor, written with UE clear. BRR is TWENTY
    /// bits here, so it is not a uint16_t - which is why brr() and
    /// set_brr() below are wider than the USART's and why the shared
    /// task stores what brr_for() hands it without narrowing.
    static bool configure(const UartFormat& f, uint32_t brr) {
        if (enabled() || !uart_format_valid(f)) {
            return false;
        }
        USART_TypeDef& r = regs();
        uint32_t cr1 = USART_CR1_TE | USART_CR1_RE;
        switch (f.bits) {
            case UartBits::seven: cr1 |= USART_CR1_M1; break;
            case UartBits::nine: cr1 |= USART_CR1_M0; break;
            default: break;
        }
        if (f.parity != UartParity::none) {
            cr1 |= USART_CR1_PCE;
            if (f.parity == UartParity::odd) {
                cr1 |= USART_CR1_PS;
            }
        }
        r.CR1 = cr1;
        r.CR2 = f.stop_bits == 2 ? USART_CR2_STOP_1 : 0u;
        r.CR3 = 0;
        r.PRESC = 0;
        r.BRR = brr & USART_BRR_LPUART;
        return true;
    }

    static uint32_t brr() { return regs().BRR & USART_BRR_LPUART; }
    static void set_brr(uint32_t v) { regs().BRR = v & USART_BRR_LPUART; }

    /// The shared task's four arithmetic questions, answered by 34.4.7.
    /// The `_over8` siblings exist so the task compiles; they can never
    /// be reached, because the task static_asserts OVER8 away on an
    /// instance whose has_oversampling8 is false.
    static constexpr std::optional<uint32_t> brr_for(uint32_t ker_hz, uint32_t baud) {
        return lpuart_brr(ker_hz, baud);
    }
    static constexpr std::optional<uint32_t> brr_for_over8(uint32_t, uint32_t) {
        return std::nullopt;
    }
    static constexpr uint32_t baud_for(uint32_t ker_hz, uint32_t reg) {
        return lpuart_actual_baud(ker_hz, reg);
    }
    static constexpr uint32_t baud_for_over8(uint32_t, uint32_t) { return 0; }
    static constexpr uint32_t min_hz_for(uint32_t baud) { return lpuart_min_hz(baud); }
    static constexpr uint32_t min_hz_for_over8(uint32_t baud) { return lpuart_min_hz(baud); }

    // ---- the fields chapter 34 shares with chapter 33 ---------------------------
    //
    // The register layout is the same and so are the enable rules; what
    // differs is only which of them EXIST, and the refusals below are
    // the LP column of table 184.

    static bool oversampling(bool over8) { return !over8; }   // there is none
    static bool oversampling() { return false; }

    static bool fifo(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR1 = on ? (regs().CR1 | USART_CR1_FIFOEN)
                        : (regs().CR1 & ~USART_CR1_FIFOEN);
        return true;
    }
    static bool fifo() { return (regs().CR1 & USART_CR1_FIFOEN) != 0u; }

    static bool fifo_thresholds(UartFifoThreshold rx, UartFifoThreshold tx) {
        if (enabled() || !uart_fifo_threshold_valid(rx) ||
            !uart_fifo_threshold_valid(tx)) {
            return false;
        }
        uint32_t cr3 = regs().CR3;
        if (rx != UartFifoThreshold::none) {
            cr3 = (cr3 & ~USART_CR3_RXFTCFG) |
                  (static_cast<uint32_t>(rx) << USART_CR3_RXFTCFG_Pos);
        }
        if (tx != UartFifoThreshold::none) {
            cr3 = (cr3 & ~USART_CR3_TXFTCFG) |
                  (static_cast<uint32_t>(tx) << USART_CR3_TXFTCFG_Pos);
        }
        regs().CR3 = cr3;
        return true;
    }

    static bool prescaler(UsartPrescaler p) {
        if (enabled() || !usart_prescaler_valid(p)) {
            return false;
        }
        regs().PRESC = static_cast<uint32_t>(p);
        return true;
    }
    static UsartPrescaler prescaler() {
        return static_cast<UsartPrescaler>(regs().PRESC & USART_PRESC_PRESCALER_Msk);
    }

    static bool swap(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = on ? (regs().CR2 | USART_CR2_SWAP) : (regs().CR2 & ~USART_CR2_SWAP);
        return true;
    }
    static bool swap() { return (regs().CR2 & USART_CR2_SWAP) != 0u; }

    static bool invert(bool tx, bool rx, bool data) {
        if (enabled()) {
            return false;
        }
        uint32_t cr2 = regs().CR2 &
                       ~(USART_CR2_TXINV | USART_CR2_RXINV | USART_CR2_DATAINV);
        if (tx) { cr2 |= USART_CR2_TXINV; }
        if (rx) { cr2 |= USART_CR2_RXINV; }
        if (data) { cr2 |= USART_CR2_DATAINV; }
        regs().CR2 = cr2;
        return true;
    }

    static bool msb_first(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = on ? (regs().CR2 | USART_CR2_MSBFIRST)
                        : (regs().CR2 & ~USART_CR2_MSBFIRST);
        return true;
    }

    static bool half_duplex(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR3 = on ? (regs().CR3 | USART_CR3_HDSEL)
                        : (regs().CR3 & ~USART_CR3_HDSEL);
        return true;
    }
    static bool half_duplex() { return (regs().CR3 & USART_CR3_HDSEL) != 0u; }

    /// 34.4.6: the LPUART's receiver samples ONCE per bit by
    /// construction - there is no three-sample majority and so no
    /// ONEBIT bit and no noise flag to lose. The verb exists so the
    /// shared task compiles; it accepts only the truth.
    static bool one_bit_sampling(bool on) { return !on; }

    static bool overrun_disable(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR3 = on ? (regs().CR3 | USART_CR3_OVRDIS)
                        : (regs().CR3 & ~USART_CR3_OVRDIS);
        return true;
    }

    static bool driver_enable(const DriverEnableConfig& c) {
        if (enabled() || !driver_enable_valid(c)) {
            return false;
        }
        regs().CR1 = (regs().CR1 & ~(USART_CR1_DEAT | USART_CR1_DEDT)) |
                     (static_cast<uint32_t>(c.assertion) << USART_CR1_DEAT_Pos) |
                     (static_cast<uint32_t>(c.deassertion) << USART_CR1_DEDT_Pos);
        uint32_t cr3 = regs().CR3 | USART_CR3_DEM;
        cr3 = c.active_low ? (cr3 | USART_CR3_DEP) : (cr3 & ~USART_CR3_DEP);
        regs().CR3 = cr3;
        return true;
    }
    static bool driver_enable_off() {
        if (enabled()) {
            return false;
        }
        regs().CR3 = regs().CR3 & ~USART_CR3_DEM;
        return true;
    }

    static bool flow_control(bool rts, bool cts) {
        if (enabled()) {
            return false;
        }
        uint32_t cr3 = regs().CR3 & ~(USART_CR3_RTSE | USART_CR3_CTSE);
        if (rts) { cr3 |= USART_CR3_RTSE; }
        if (cts) { cr3 |= USART_CR3_CTSE; }
        regs().CR3 = cr3;
        return true;
    }

    static bool mute_mode(const MuteConfig& c) {
        if (enabled()) {
            return false;
        }
        uint32_t cr1 = regs().CR1 | USART_CR1_MME;
        cr1 = c.wake == MuteWake::address_mark ? (cr1 | USART_CR1_WAKE)
                                               : (cr1 & ~USART_CR1_WAKE);
        regs().CR1 = cr1;
        uint32_t cr2 = regs().CR2 & ~(USART_CR2_ADD | USART_CR2_ADDM7);
        if (c.address_7bit) {
            cr2 |= USART_CR2_ADDM7;
        }
        regs().CR2 = cr2 | (static_cast<uint32_t>(c.address) << USART_CR2_ADD_Pos);
        return true;
    }
    static bool mute_mode_off() {
        if (enabled()) {
            return false;
        }
        regs().CR1 = regs().CR1 & ~USART_CR1_MME;
        return true;
    }
    static bool muted() { return (regs().ISR & UsartFlag::rwu) != 0u; }

    static bool character_match(uint8_t ch) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = (regs().CR2 & ~USART_CR2_ADD) |
                     (static_cast<uint32_t>(ch) << USART_CR2_ADD_Pos);
        return true;
    }

    /// 34.4.14: UESM + WUS + WUFIE, and the direct EXTI line. The same
    /// obligations the USART's carries, and one more that matters more
    /// here: on LSE this instance can serve a whole frame with the rest
    /// of the chip stopped, which is the reason the peripheral exists.
    static bool wake_from_stop(UsartWakeSource s) {
        if (enabled()) {
            return false;
        }
        if (s == UsartWakeSource::none) {
            regs().CR1 = regs().CR1 & ~USART_CR1_UESM;
            regs().CR3 = regs().CR3 & ~(USART_CR3_WUS | USART_CR3_WUFIE);
            return true;
        }
        regs().CR1 = regs().CR1 | USART_CR1_UESM;
        regs().CR3 = (regs().CR3 & ~USART_CR3_WUS) | USART_CR3_WUFIE |
                     (static_cast<uint32_t>(s) << USART_CR3_WUS_Pos);
        return true;
    }
    static bool wake_line(bool on) { return Exti::interrupt(exti_line, on); }
    static bool wake_line() { return Exti::interrupt(exti_line); }

    // ---- requests, data, flags ----------------------------------------------------

    static void request(UsartRequest r) { regs().RQR = static_cast<uint32_t>(r); }
    static void send_break() { request(UsartRequest::send_break); }

    static uint32_t status() { return regs().ISR; }
    static bool flag(uint32_t mask) { return (regs().ISR & mask) != 0u; }
    static void clear_flags(uint32_t icr_mask) { regs().ICR = icr_mask; }
    static uint8_t read_data() { return static_cast<uint8_t>(regs().RDR); }
    static uint16_t read_word() { return static_cast<uint16_t>(regs().RDR & 0x1FFu); }
    static void write_data(uint8_t b) { regs().TDR = b; }
    static void write_word(uint16_t v) { regs().TDR = v & 0x1FFu; }

    static volatile void* tx_data_address() { return &regs().TDR; }
    static volatile void* rx_data_address() { return &regs().RDR; }

    static bool dma_transmit(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR3 = on ? (regs().CR3 | USART_CR3_DMAT) : (regs().CR3 & ~USART_CR3_DMAT);
        return true;
    }
    static bool dma_receive(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR3 = on ? (regs().CR3 | USART_CR3_DMAR) : (regs().CR3 & ~USART_CR3_DMAR);
        return true;
    }

    /// The DMAMUX rows of table 55, published by the peripheral that
    /// owns them (the standing ruling; no header of this pack spells
    /// them). LPUART1's rows are the ones the TABLE CALLS `LPUART_RX`
    /// and `LPUART_TX` WITHOUT AN INDEX - 14 and 15, low down among the
    /// I2Cs and the SPIs and nowhere near the USARTs' block - while
    /// LPUART2 sits at 64 and 65 with the rest of the G0B1 class's
    /// additions. Reading the table by shape rather than by name is how
    /// this pair first came out as 22 and 23, which are TIM1_CH3 and
    /// TIM1_CH4.
    static constexpr uint8_t dma_rx_request() { return n == 1 ? 14 : 64; }
    static constexpr uint8_t dma_tx_request() { return n == 1 ? 15 : 65; }

    static void rxne_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR1 = on ? (regs().CR1 | USART_CR1_RXNEIE_RXFNEIE)
                        : (regs().CR1 & ~USART_CR1_RXNEIE_RXFNEIE);
    }
    static void txe_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR1 = on ? (regs().CR1 | USART_CR1_TXEIE_TXFNFIE)
                        : (regs().CR1 & ~USART_CR1_TXEIE_TXFNFIE);
    }
    static bool txe_interrupt() { return (regs().CR1 & USART_CR1_TXEIE_TXFNFIE) != 0u; }

    static void interrupts(uint32_t cr1_mask, bool on) {
        InterruptGuard guard;
        regs().CR1 = on ? (regs().CR1 | cr1_mask) : (regs().CR1 & ~cr1_mask);
    }
    static uint32_t interrupts() { return regs().CR1; }

    static void error_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_EIE) : (regs().CR3 & ~USART_CR3_EIE);
    }
    static void cts_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_CTSIE) : (regs().CR3 & ~USART_CR3_CTSIE);
    }
    static void rx_threshold_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_RXFTIE) : (regs().CR3 & ~USART_CR3_RXFTIE);
    }
    static void tx_threshold_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_TXFTIE) : (regs().CR3 & ~USART_CR3_TXFTIE);
    }

    static void reset() {
        constexpr UsartBusClock bc = lpuart_bus_clock(n);
        if constexpr (bc.apb2) {
            RCC->APBRSTR2 = RCC->APBRSTR2 | bc.mask;
            RCC->APBRSTR2 = RCC->APBRSTR2 & ~bc.mask;
        } else {
            RCC->APBRSTR1 = RCC->APBRSTR1 | bc.mask;
            RCC->APBRSTR1 = RCC->APBRSTR1 & ~bc.mask;
        }
    }
};

/// The task over an LPUART instance - stm32g0/usart.hpp's UartTask, with
/// this chapter's resource under it. Every verb, every default and every
/// engine slot is the same; what changes is what the resource answers.
template <uint8_t n, UartPins pins, uint32_t rx_size = 64, uint32_t tx_size = 256,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine,
          UartOptions opts = UartOptions{}>
using LpUart = UartTask<Lpuart<n>, pins, rx_size, tx_size, TxEngine, RxEngine, opts>;

} // namespace brio
