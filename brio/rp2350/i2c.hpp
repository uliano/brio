/*
 * i2c.hpp
 *
 * The I2C (datasheet 12.2: two Synopsys DW_apb_i2c controllers, IP
 * version 2.03a). The chapter's driver is NOT here: the DW_apb_i2c is a
 * licensed design this chip shares with others, so the resource, the host
 * engine and the client are written once in the IP stratum
 * (brio/dw_apb_i2c/i2c.hpp, docs/dw_apb_i2c/README.md) and this file is
 * what that file asks of a family -
 *
 *  - the pin table of 9.4 with its function code, and the pad setup
 *    12.2.1.3 asks for;
 *  - `Rp2350DwApbI2c`, the CHIP TRAITS: the register block and where each
 *    instance's sits, the reset bits of 7.5, the interrupt lines of 3.2,
 *    the atomic set/clear aliases of 2.1.3, the DREQ numbers of 12.6.4.1,
 *    the pad type with the function code that routes an I2C to it and the
 *    five verbs the unstick drives an open-drain line with (3.1.3: the
 *    SIO output enable is the drive, over an output register holding a
 *    zero), the microsecond ruler the unstick paces itself by, and WHICH
 *    RATE IS ic_clk - clk_sys (12.2.1.2), so the counts come from the
 *    app's own clock and never from a second statement of the rate;
 *  - the PUBLIC NAMES: `DwApbI2c<n>` the resource, `I2cHost<n, pins,
 *    ...>` the engine util/i2c_bus.hpp's I2cBus drives and `I2cClient<n,
 *    pins>` the other end, this chip's aliases of the three IP templates.
 *
 * THE PINS ARE ONE COLUMN AND NOT TWO. Every fourth pad from GP0 carries
 * an I2C signal - SDA on the even pads, SCL on the odd ones, the instance
 * alternating in pairs - so I2C0 has SDA on GP0, 4, 8, ... 44 and SCL on
 * 1, 5, 9, ... 45, and I2C1 has SDA on 2, 6, 10, ... 46 and SCL on 3, 7,
 * 11, ... 47. That is the RP2040's rule over a longer bank: forty-eight
 * pads on the QFN-80 against thirty, so twelve SDA pads and twelve SCL
 * pads per instance against eight and eight. WHAT THE UART GAINED HERE,
 * THIS BLOCK DID NOT: the UART's flow-control pads carry data too under a
 * second function code, and 9.4 gives the I2C exactly one I2C entry per
 * pad, all forty-eight of them function 3. So `I2cPins` stays a pair of
 * pad numbers, as on the RP2040, and needs no column beside them.
 *
 * THE PADS COME UP ISOLATED (9.7 for the latch, 9.11 for the reset value
 * 0x116, which has it set and the input buffer off), so a pad set up as
 * on the RP2040 would answer nothing. Every configuring verb of pin.hpp
 * writes the whole pad register with ISO CLEAR, which is what makes the
 * two lines reach the block; the setup 12.2.1.3 asks for - pull-up, slew
 * limited, Schmitt trigger - is `i2c_pad_config` below, and the board's
 * own pull-ups do the pulling.
 *
 * THE BANK IS TWO WORDS, which is the one thing `I2cPad` does not share
 * with its RP2040 twin: SIO holds GPIO0..31 in GPIO_OE and GPIO32..47 in
 * GPIO_HI_OE, so an open-drain drive by hand picks the half the pad falls
 * in. `Pin<n>::mask` is the pin's bit in ITS OWN half and `high_half`
 * says which, so the choice is an `if constexpr` and costs nothing.
 *
 * THE LINE. The thirteen sources of an instance combine into ONE
 * interrupt line (I2C0_IRQ = 36, I2C1_IRQ = 37, datasheet 3.2); the app
 * binds isr_i2c0 / isr_i2c1 to the instance's isr() or service() body,
 * and those names are bound on BOTH architectures - a Cortex-M vector
 * table on one half, Hazard3's own dispatch on the other - because the
 * interrupt numbering is shared (3.8.4.2). Which is also why
 * `Interrupts` below is the controller core.hpp names and never an NVIC.
 *
 * THE BUS CLEAR of 12.2.13 (nine clocks and a STOP by the master) has no
 * enable in this chip's IC_CON, whose writable bits are 0x7FF and stop at
 * the eleventh - so the unstick is by hand, which is why the traits carry
 * the open-drain verbs.
 *
 * THE RULER THE UNSTICK PACES ITSELF BY is rp2350/delay.hpp, which counts
 * THE PLATFORM TIMER and not CPU cycles, because that counter is the one
 * ruler both architectures have (the file says why). It follows that a
 * program which calls `unstick()` must have started that counter -
 * `Mtime::start(clock)`, which the Hazard3 half's ticker already does and
 * the Cortex-M33 half's does not. With the counter stopped every wait is
 * REFUSED and spends no time, so the nine pulses go out at the speed of
 * two register writes: legal on the wire, far faster than a bus, and
 * measurable, which is how the reference suite states it.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "dw_apb_i2c/i2c.hpp"
#include "rp2350/core.hpp"
#include "rp2350/delay.hpp"
#include "rp2350/dma_engine.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/resets.hpp"

namespace brio {

// ---- the pads --------------------------------------------------------------------

/// Which pads carry the two lines: each the instance's own under
/// function 3 (9.4). One column, so a pad number is the whole answer.
struct I2cPins {
    uint8_t scl = 0xFFu;
    uint8_t sda = 0xFFu;
};

/// SDA is an EVEN pad, and which instance it belongs to is the pad number
/// halved: GP0/GP1 are I2C0's, GP2/GP3 I2C1's, and so on in pairs to
/// GP47.
///
/// The PACKAGE is not asked here: every pad's registers exist on both
/// packages and only the bond wire does not, so a pad the QFN-60 has not
/// got is refused one level down, by `Pin<n>`'s own static_assert, which
/// says so in those words.
constexpr bool i2c_sda_pin(uint8_t n, uint8_t p) {
    return p < gpio_count_max && (p & 1u) == 0u && ((p >> 1) & 1u) == n;
}
/// And SCL is the odd pad of the same pair.
constexpr bool i2c_scl_pin(uint8_t n, uint8_t p) {
    return p < gpio_count_max && (p & 1u) == 1u && ((p >> 1) & 1u) == n;
}
constexpr bool i2c_pins_valid(uint8_t n, const I2cPins& p) {
    return n < 2u && i2c_sda_pin(n, p.sda) && i2c_scl_pin(n, p.scl);
}

/// The pad configuration 12.2.1.3 asks for: pull-up, slew limited,
/// Schmitt trigger. The board's pull-ups do the pulling; the pad's is a
/// courtesy, and the note beside that list says so. Writing this value IS
/// taking the pad out of isolation (pin.hpp's `pad_value`).
inline constexpr PinConfig i2c_pad_config{.pull = PinPull::up, .drive = PinDrive::ma4, .slew_fast = false,
                                          .schmitt = true, .input_enable = true};

/**
 * One pad of an I2C, with the two verbs the unstick adds to a Pin's: an
 * open-drain line is driven by its OUTPUT ENABLE alone, over an output
 * register the caller has already cleared (3.1.3's GPIO_OE_SET and
 * GPIO_OE_CLR). This chip's bank is TWO words, so each verb picks the
 * half its pad falls in - the one place where the RP2040's I2cPad would
 * have written the wrong register for GP32..GP47.
 */
template <uint8_t pin>
struct I2cPad : Pin<pin> {
    static void drive_low() {
        if constexpr (Pin<pin>::high_half) {
            Gpio::oe_set_hi(Pin<pin>::mask);
        } else {
            Gpio::oe_set(Pin<pin>::mask);
        }
    }
    static void release_drive() {
        if constexpr (Pin<pin>::high_half) {
            Gpio::oe_clear_hi(Pin<pin>::mask);
        } else {
            Gpio::oe_clear(Pin<pin>::mask);
        }
    }
};

// ---- what this chip tells the DW_apb_i2c driver -----------------------------------

/**
 * The chip traits brio/dw_apb_i2c/i2c.hpp is instantiated with: every
 * fact about the DW_apb_i2c that is the RP2350's rather than Synopsys's,
 * and nothing else. Nothing above this file ever names it.
 */
struct Rp2350DwApbI2c {
    Rp2350DwApbI2c() = delete;

    using Regs = I2C0_Type;
    /// What the controller calls a line. `Interrupts` below is the
    /// controller itself, which on this chip is `brio::Irq` - an NVIC
    /// under one architecture and Hazard3's own under the other, named
    /// once in core.hpp. The qualified spelling is what keeps this
    /// member from shadowing it.
    using Irq = IRQn_Type;
    using Interrupts = ::brio::Irq;
    using Pins = I2cPins;
    using DmaRequest = Dreq;
    using SpinRate = DelayRate;

    /// I2C0 and I2C1, both with 16-deep FIFOs (12.2.1: a 16-element
    /// transmit buffer and a 16-element receive buffer - the component
    /// parameter register that would report them is not implemented and
    /// reads zero).
    static constexpr uint8_t instances = 2;
    static constexpr uint8_t fifo_depth = 16;

    template <uint8_t i>
    static Regs& regs() { return *(i == 0 ? I2C0 : I2C1); }
    template <uint8_t i>
    static constexpr Irq irq() { return i == 0 ? I2C0_IRQ_IRQn : I2C1_IRQ_IRQn; }

    /// The instance's bit in the subsystem reset controller (7.5). NOT
    /// the RP2040's bits: this chip's controller governs twenty-nine
    /// blocks against twenty-seven and everything above the third has
    /// moved, so the numbers come from this chip's own header through
    /// resets.hpp.
    template <uint8_t i>
    static constexpr uint32_t reset_bit() { return i == 0 ? ResetBlock::i2c0 : ResetBlock::i2c1; }

    /// The three verbs over it, FORWARDERS and declared as such: each is
    /// one call with a constant argument, and the function the compiler
    /// outlines must be the resource's verb rather than a frame of this
    /// file's.
    template <uint8_t i>
    [[gnu::always_inline]] static bool reset() { return Resets::cycle(reset_bit<i>()); }
    template <uint8_t i>
    [[gnu::always_inline]] static bool released() { return Resets::released(reset_bit<i>()); }
    template <uint8_t i>
    [[gnu::always_inline]] static void hold() { Resets::hold(reset_bit<i>()); }

    /// 12.6.4.1: the two data requests of an instance. NOT the RP2040's
    /// numbers - a third PIO and twelve PWM slices moved every row above
    /// the first PIO's, so the I2Cs sit at 44..47 where they sat at
    /// 32..35 (dma_engine.hpp).
    template <uint8_t i>
    static constexpr DmaRequest tx_request() { return i == 0 ? Dreq::i2c0_tx : Dreq::i2c1_tx; }
    template <uint8_t i>
    static constexpr DmaRequest rx_request() { return i == 0 ? Dreq::i2c0_rx : Dreq::i2c1_rx; }

    /// The atomic register aliases of 2.1.3: one bus write, no
    /// read-modify-write.
    static void set_bits(volatile uint32_t& reg, uint32_t bits) { hw_set(reg, bits); }
    static void clear_bits(volatile uint32_t& reg, uint32_t bits) { hw_clear(reg, bits); }

    /// The pin table above, and the two halves of a pin set.
    static constexpr bool pins_valid(uint8_t n, const Pins& p) { return i2c_pins_valid(n, p); }
    static constexpr uint8_t scl_pad(const Pins& p) { return p.scl; }
    static constexpr uint8_t sda_pad(const Pins& p) { return p.sda; }

    /// The pad as a type, the function code that routes an I2C to it
    /// (9.4: F3 on every I2C pad, and there is no second column here)
    /// and the electrical setup both lines want; `open_drain_pull` is
    /// what a line released by hand idles at while the unstick owns it -
    /// the pull-UP, which is also why erratum RP2350-E9 cannot colour
    /// what the unstick reads.
    template <uint8_t pin>
    using Pad = I2cPad<pin>;
    static constexpr PinFunction pad_function = PinFunction::i2c;
    static constexpr const PinConfig& pad_config = i2c_pad_config;
    static constexpr PinPull open_drain_pull = PinPull::up;

    /// Two engines of one host must not share a channel: on this chip a
    /// request is a field any channel takes, so the CHANNEL is the
    /// identity (rp2350/dma_engine.hpp).
    template <typename Tx, typename Rx>
    static constexpr bool engines_distinct() { return uart_engines_distinct<Tx, Rx>(); }

    /// The microsecond ruler the unstick paces its half-bits by
    /// (rp2350/delay.hpp, the platform timer of 3.1.8 - the one ruler
    /// both architectures have). The rate is asked for the contract's
    /// sake and for the refusal of a zero one; the counter itself is
    /// what measures the wait, and the file header says what a program
    /// that has not started it gets.
    static constexpr SpinRate spin_rate(uint32_t hz) { return delay_rate(hz); }
    static void spin_us(SpinRate rate, uint32_t us) { (void)delay_us(rate, us); }

    /// ic_clk is clk_sys (12.2.1.2: every clock in the controller is
    /// clk_sys, and the I2C clock is divided down from it inside the
    /// block) - the app's own clock rate, so the counts never come from
    /// a second statement of it.
    template <typename Clock>
    static constexpr uint32_t ic_clk_hz(Clock clock) { return clock_hz(clock); }
};

static_assert(DwApbI2cChip<Rp2350DwApbI2c>);

// ---- the public names --------------------------------------------------------------

/// The resource: which instance, where its registers are, its reset bit,
/// its interrupt line, its two data requests, and the whole of the
/// DW_apb_i2c's programmer's model.
template <uint8_t n>
using DwApbI2c = DwApbI2cBlock<Rp2350DwApbI2c, n>;

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
 * The same source builds for both architectures: `isr_i2c0` is a
 * Cortex-M vector on one half and a Hazard3 dispatch entry on the other.
 *
 * The two engine slots take rp2350/dma.hpp's DmaTxEngine<ch, uint16_t>
 * and DmaRxEngine<ch> on any two of the sixteen channels, both or
 * neither.
 */
template <uint8_t n, I2cPins pins, typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
using I2cHost = DwApbI2cHost<Rp2350DwApbI2c, n, pins, TxEngine, RxEngine>;

/// The target side, over DwApbI2c<n>: a polled surface and one ISR body
/// the app binds to isr_i2c0 / isr_i2c1.
template <uint8_t n, I2cPins pins>
using I2cClient = DwApbI2cClient<Rp2350DwApbI2c, n, pins>;

// ---- this chip's device description against the IP's own words ----------------------

// The IP file spells the block's bits itself, because it may not read a
// vendor header; here is where the two are held against each other. This
// chip's i2c.h is its own file in its own include root, and these are ITS
// macros - that they turn out to agree with the other chip's, register
// for register, is the measurement that made an IP stratum worth having
// and not an assumption this file may make.
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
// And the bus clear of 12.2.13 has no enable here: IC_CON's writable bits
// are 0x7FF and stop at the eleventh, so there is no bit to ask for it.
static_assert(I2C_IC_CON_BITS == 0x7FFu && I2C_IC_CON_RESET == 0x65u);
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

// 9.4's one column, over the whole of the larger package - the four pads
// above the RP2040's bank included - and a pin set from the wrong
// instance.
static_assert(i2c_pins_valid(0, {.scl = 13, .sda = 12}) && i2c_pins_valid(1, {.scl = 15, .sda = 14}));
static_assert(i2c_sda_pin(0, 0) && i2c_scl_pin(0, 1) && i2c_sda_pin(1, 2) && i2c_scl_pin(1, 3));
static_assert(i2c_sda_pin(0, 44) && i2c_scl_pin(0, 45) && i2c_sda_pin(1, 46) && i2c_scl_pin(1, 47));
static_assert(!i2c_sda_pin(0, 2) && !i2c_scl_pin(1, 13) && !i2c_sda_pin(1, 48) &&
              !i2c_pins_valid(0, {.scl = 12, .sda = 13}));

} // namespace brio
