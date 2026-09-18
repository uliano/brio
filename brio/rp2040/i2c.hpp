/*
 * i2c.hpp
 *
 * The I2C (datasheet 4.3: two Synopsys DW_apb_i2c controllers). The
 * chapter's driver is NOT here: the DW_apb_i2c is a licensed design this
 * chip shares with others, so the resource, the host engine and the
 * client are written once in the IP stratum (brio/dw_apb_i2c/i2c.hpp,
 * docs/dw_apb_i2c/README.md) and this file is what that file asks of a
 * family -
 *
 *  - the pin table of 2.19.2 (table 279) with its function code, and
 *    the pad setup 4.3.1.3 asks for;
 *  - `Rp2040DwApbI2c`, the CHIP TRAITS: the register block and where
 *    each instance's sits, the reset bits of 2.14, the NVIC lines of
 *    2.3.2, the atomic set/clear aliases of 2.1.2, the DREQ numbers of
 *    table 119, the pad type with the function code that routes an I2C
 *    to it and the five verbs the unstick drives an open-drain line
 *    with (2.19: the SIO output enable is the drive, over an output
 *    register holding a zero), the microsecond ruler the unstick paces
 *    itself by, and WHICH RATE IS ic_clk - clk_sys (4.3.14.1), so the
 *    counts come from the app's own clock and never from a second
 *    statement of the rate;
 *  - the PUBLIC NAMES: `DwApbI2c<n>` the resource, `I2cHost<n, pins,
 *    ...>` the engine util/i2c_bus.hpp's I2cBus drives and
 *    `I2cClient<n, pins>` the other end, this chip's aliases of the
 *    three IP templates.
 *
 * THE PINS are fixed per instance under function 3 (2.19.2, table 279):
 * I2C0 has SDA on GPIO 0, 4, 8, 12, 16, 20, 24, 28 and SCL on 1, 5, 9,
 * 13, 17, 21, 25, 29; I2C1 has SDA on 2, 6, 10, 14, 18, 22, 26 and SCL
 * on 3, 7, 11, 15, 19, 23, 27. `i2c_pins_valid` is that table, and a pin
 * set that is not in it does not compile. The pads take the pull-up, the
 * slew limit and the Schmitt trigger 4.3.1.3 asks for; the board's own
 * pull-ups do the pulling.
 *
 * THE LINE. The thirteen sources of an instance combine into ONE NVIC
 * line (I2C0_IRQ, I2C1_IRQ, table 2.3.2); the app binds isr_i2c0 /
 * isr_i2c1 to the instance's isr() or service() body.
 *
 * THE BUS CLEAR of 4.3.13 (nine clocks and a STOP) has no enable in this
 * chip's IC_CON: the unstick is by hand, which is why the traits carry
 * the open-drain verbs.
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "dw_apb_i2c/i2c.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/resets.hpp"

namespace brio {

// ---- the pads --------------------------------------------------------------------

/// Which pins carry the two lines: each the instance's own under
/// function 3 (table 279).
struct I2cPins {
    uint8_t scl = 0xFFu;
    uint8_t sda = 0xFFu;
};

constexpr bool i2c_sda_pin(uint8_t n, uint8_t p) { return p < gpio_count && (p & 1u) == 0u && ((p >> 1) & 1u) == n; }
constexpr bool i2c_scl_pin(uint8_t n, uint8_t p) { return p < gpio_count && (p & 1u) == 1u && ((p >> 1) & 1u) == n; }
constexpr bool i2c_pins_valid(uint8_t n, const I2cPins& p) {
    return n < 2u && i2c_sda_pin(n, p.sda) && i2c_scl_pin(n, p.scl);
}

/// The pad configuration 4.3.1.3 asks for: pull-up, slew limited,
/// Schmitt trigger. The board's pull-ups do the pulling; the pad's is
/// a courtesy.
inline constexpr PinConfig i2c_pad_config{.pull = PinPull::up, .drive = PinDrive::ma4, .slew_fast = false,
                                          .schmitt = true, .input_enable = true};

/**
 * One pad of an I2C, with the two verbs the unstick adds to a Pin's: an
 * open-drain line is driven by its OUTPUT ENABLE alone, over an output
 * register the caller has already cleared (2.19: SIO's GPIO_OE_SET and
 * GPIO_OE_CLR).
 */
template <uint8_t pin>
struct I2cPad : Pin<pin> {
    static void drive_low() { Gpio::oe_set(Pin<pin>::mask); }
    static void release_drive() { Gpio::oe_clear(Pin<pin>::mask); }
};

// ---- what this chip tells the DW_apb_i2c driver -----------------------------------

/**
 * The chip traits brio/dw_apb_i2c/i2c.hpp is instantiated with: every
 * fact about the DW_apb_i2c that is the RP2040's rather than Synopsys's,
 * and nothing else. Nothing above this file ever names it.
 */
struct Rp2040DwApbI2c {
    Rp2040DwApbI2c() = delete;

    using Regs = I2C0_Type;
    using Irq = IRQn_Type;
    using Interrupts = Nvic;
    using Pins = I2cPins;
    using DmaRequest = Dreq;
    using SpinRate = DelayRate;

    /// I2C0 and I2C1, both with 16-deep FIFOs (the chapter's word: the
    /// component parameter register is not implemented here).
    static constexpr uint8_t instances = 2;
    static constexpr uint8_t fifo_depth = 16;

    template <uint8_t i>
    static Regs& regs() { return *(i == 0 ? I2C0 : I2C1); }
    template <uint8_t i>
    static constexpr Irq irq() { return i == 0 ? I2C0_IRQ_IRQn : I2C1_IRQ_IRQn; }

    /// The instance's bit in the subsystem reset controller (2.14).
    template <uint8_t i>
    static constexpr uint32_t reset_bit() { return i == 0 ? ResetBlock::i2c0 : ResetBlock::i2c1; }

    /// The three verbs over it, FORWARDERS and declared as such: each is
    /// one call with a constant argument, and the function the compiler
    /// outlines must be the resource's verb rather than a frame of this
    /// file's - which is what keeps a program's image where it was.
    template <uint8_t i>
    [[gnu::always_inline]] static bool reset() { return Resets::cycle(reset_bit<i>()); }
    template <uint8_t i>
    [[gnu::always_inline]] static bool released() { return Resets::released(reset_bit<i>()); }
    template <uint8_t i>
    [[gnu::always_inline]] static void hold() { Resets::hold(reset_bit<i>()); }

    /// Table 119: the two data requests of an instance.
    template <uint8_t i>
    static constexpr DmaRequest tx_request() { return i == 0 ? Dreq::i2c0_tx : Dreq::i2c1_tx; }
    template <uint8_t i>
    static constexpr DmaRequest rx_request() { return i == 0 ? Dreq::i2c0_rx : Dreq::i2c1_rx; }

    /// The atomic register aliases of 2.1.2: one bus write, no
    /// read-modify-write.
    static void set_bits(volatile uint32_t& reg, uint32_t bits) { hw_set(reg, bits); }
    static void clear_bits(volatile uint32_t& reg, uint32_t bits) { hw_clear(reg, bits); }

    /// The pin table above, and the two halves of a pin set.
    static constexpr bool pins_valid(uint8_t n, const Pins& p) { return i2c_pins_valid(n, p); }
    static constexpr uint8_t scl_pad(const Pins& p) { return p.scl; }
    static constexpr uint8_t sda_pad(const Pins& p) { return p.sda; }

    /// The pad as a type, the function code that routes an I2C to it
    /// (2.19.2: F3 on every I2C pin) and the electrical setup both lines
    /// want; `open_drain_pull` is what a line released by hand idles at
    /// while the unstick owns it.
    template <uint8_t pin>
    using Pad = I2cPad<pin>;
    static constexpr PinFunction pad_function = PinFunction::i2c;
    static constexpr const PinConfig& pad_config = i2c_pad_config;
    static constexpr PinPull open_drain_pull = PinPull::up;

    /// Two engines of one host must not share a channel: on this chip a
    /// request is a field any channel takes, so the CHANNEL is the
    /// identity (rp2040/dma_engine.hpp).
    template <typename Tx, typename Rx>
    static constexpr bool engines_distinct() { return uart_engines_distinct<Tx, Rx>(); }

    /// The microsecond ruler the unstick paces its half-bits by
    /// (rp2040/delay.hpp, SysTick's VAL).
    static constexpr SpinRate spin_rate(uint32_t hz) { return delay_rate(hz); }
    static void spin_us(SpinRate rate, uint32_t us) { (void)delay_us(rate, us); }

    /// ic_clk is clk_sys (4.3.14.1): the app's own clock rate, so the
    /// counts never come from a second statement of it.
    template <typename Clock>
    static constexpr uint32_t ic_clk_hz(Clock clock) { return clock_hz(clock); }
};

static_assert(DwApbI2cChip<Rp2040DwApbI2c>);

// ---- the public names --------------------------------------------------------------

/// The resource: which instance, where its registers are, its reset bit,
/// its NVIC line, its two data requests, and the whole of the
/// DW_apb_i2c's programmer's model.
template <uint8_t n>
using DwApbI2c = DwApbI2cBlock<Rp2040DwApbI2c, n>;

/**
 * The engine util/i2c_bus.hpp's I2cBus drives, over DwApbI2c<n>.
 *
 *   constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
 *   using Bus = brio::I2cHost<0, pins>;
 *   using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough,
 *                            brio::ticks_from_ms<P>(20)>;
 *
 *   extern "C" void isr_i2c0() {
 *       if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
 *   }
 *
 * The two engine slots take rp2040/dma.hpp's DmaTxEngine<ch, uint16_t>
 * and DmaRxEngine<ch> on any two channels, both or neither.
 */
template <uint8_t n, I2cPins pins, typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
using I2cHost = DwApbI2cHost<Rp2040DwApbI2c, n, pins, TxEngine, RxEngine>;

/// The target side, over DwApbI2c<n>: a polled surface and one ISR body
/// the app binds to isr_i2c0 / isr_i2c1.
template <uint8_t n, I2cPins pins>
using I2cClient = DwApbI2cClient<Rp2040DwApbI2c, n, pins>;

// ---- this chip's device description against the IP's own words ----------------------

// The IP file spells the block's bits itself, because it may not read a
// vendor header; here is where the two are held against each other.
static_assert(I2cControl::host_mode == I2C_IC_CON_MASTER_MODE_BITS &&
              I2cControl::speed_lsb == I2C_IC_CON_SPEED_LSB &&
              I2cControl::speed_bits == I2C_IC_CON_SPEED_BITS &&
              I2cControl::ten_bit_client == I2C_IC_CON_IC_10BITADDR_SLAVE_BITS &&
              I2cControl::ten_bit_host == I2C_IC_CON_IC_10BITADDR_MASTER_BITS &&
              I2cControl::restart_enable == I2C_IC_CON_IC_RESTART_EN_BITS &&
              I2cControl::client_disable == I2C_IC_CON_IC_SLAVE_DISABLE_BITS &&
              I2cControl::stop_det_if_addressed == I2C_IC_CON_STOP_DET_IFADDRESSED_BITS &&
              I2cControl::tx_empty_late == I2C_IC_CON_TX_EMPTY_CTRL_BITS &&
              I2cControl::hold_when_rx_full == I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_BITS &&
              I2cControl::speed_standard == I2C_IC_CON_SPEED_VALUE_STANDARD &&
              I2cControl::speed_fast == I2C_IC_CON_SPEED_VALUE_FAST);
static_assert(I2cData::byte == I2C_IC_DATA_CMD_DAT_BITS && I2cData::read == I2C_IC_DATA_CMD_CMD_BITS &&
              I2cData::stop == I2C_IC_DATA_CMD_STOP_BITS && I2cData::restart == I2C_IC_DATA_CMD_RESTART_BITS &&
              I2cData::first_data_byte == I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS);
static_assert(I2cEnable::enable == I2C_IC_ENABLE_ENABLE_BITS && I2cEnable::abort == I2C_IC_ENABLE_ABORT_BITS &&
              I2cEnable::command_block == I2C_IC_ENABLE_TX_CMD_BLOCK_BITS &&
              I2cEnableStatus::running == I2C_IC_ENABLE_STATUS_IC_EN_BITS &&
              I2cEnableStatus::client_disabled_while_busy == I2C_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY_BITS &&
              I2cEnableStatus::client_rx_data_lost == I2C_IC_ENABLE_STATUS_SLV_RX_DATA_LOST_BITS);
static_assert(I2cFlag::activity == I2C_IC_STATUS_ACTIVITY_BITS && I2cFlag::tx_not_full == I2C_IC_STATUS_TFNF_BITS &&
              I2cFlag::tx_empty == I2C_IC_STATUS_TFE_BITS && I2cFlag::rx_not_empty == I2C_IC_STATUS_RFNE_BITS &&
              I2cFlag::rx_full == I2C_IC_STATUS_RFF_BITS && I2cFlag::host_active == I2C_IC_STATUS_MST_ACTIVITY_BITS &&
              I2cFlag::client_active == I2C_IC_STATUS_SLV_ACTIVITY_BITS);
static_assert(I2cInterrupt::rx_under == I2C_IC_INTR_MASK_M_RX_UNDER_BITS &&
              I2cInterrupt::rx_over == I2C_IC_INTR_MASK_M_RX_OVER_BITS &&
              I2cInterrupt::rx_full == I2C_IC_INTR_MASK_M_RX_FULL_BITS &&
              I2cInterrupt::tx_over == I2C_IC_INTR_MASK_M_TX_OVER_BITS &&
              I2cInterrupt::tx_empty == I2C_IC_INTR_MASK_M_TX_EMPTY_BITS &&
              I2cInterrupt::rd_req == I2C_IC_INTR_MASK_M_RD_REQ_BITS &&
              I2cInterrupt::tx_abrt == I2C_IC_INTR_MASK_M_TX_ABRT_BITS &&
              I2cInterrupt::rx_done == I2C_IC_INTR_MASK_M_RX_DONE_BITS &&
              I2cInterrupt::activity == I2C_IC_INTR_MASK_M_ACTIVITY_BITS &&
              I2cInterrupt::stop_det == I2C_IC_INTR_MASK_M_STOP_DET_BITS &&
              I2cInterrupt::start_det == I2C_IC_INTR_MASK_M_START_DET_BITS &&
              I2cInterrupt::gen_call == I2C_IC_INTR_MASK_M_GEN_CALL_BITS &&
              I2cInterrupt::restart_det == I2C_IC_INTR_MASK_M_RESTART_DET_BITS);
static_assert(I2cAbort::addr_noack == (I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS |
                                       I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_BITS |
                                       I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_BITS |
                                       I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_BITS) &&
              I2cAbort::data_noack == I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS &&
              I2cAbort::arb_lost == I2C_IC_TX_ABRT_SOURCE_ARB_LOST_BITS &&
              I2cAbort::user_abort == I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_BITS &&
              I2cAbort::client_flush == I2C_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_BITS &&
              I2cAbort::client_arb_lost == I2C_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST_BITS &&
              I2cAbort::client_read_cmd == I2C_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX_BITS);
static_assert(I2cHoldField::tx == I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS &&
              I2cHoldField::rx == I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_BITS &&
              I2cHoldField::rx_lsb == I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_LSB &&
              I2cAddressField::address == I2C_IC_TAR_IC_TAR_BITS &&
              I2cAddressField::address == I2C_IC_SAR_IC_SAR_BITS &&
              I2cAddressField::special == I2C_IC_TAR_SPECIAL_BITS &&
              I2cLevelField::count == I2C_IC_TXFLR_TXFLR_BITS &&
              I2cLevelField::count == I2C_IC_RXFLR_RXFLR_BITS &&
              I2cDmaControl::tx == I2C_IC_DMA_CR_TDMAE_BITS && I2cDmaControl::rx == I2C_IC_DMA_CR_RDMAE_BITS &&
              I2cDmaControl::level == I2C_IC_DMA_TDLR_DMATDL_BITS &&
              I2cDmaControl::level == I2C_IC_DMA_RDLR_DMARDL_BITS);

// Table 279's two columns, and a pin set from the wrong instance.
static_assert(i2c_pins_valid(0, {.scl = 13, .sda = 12}) && i2c_pins_valid(1, {.scl = 15, .sda = 14}));
static_assert(i2c_sda_pin(0, 0) && i2c_sda_pin(0, 28) && i2c_scl_pin(0, 29) && i2c_sda_pin(1, 26) && i2c_scl_pin(1, 27));
static_assert(!i2c_sda_pin(0, 2) && !i2c_scl_pin(1, 13) && !i2c_pins_valid(0, {.scl = 12, .sda = 13}));

} // namespace brio
