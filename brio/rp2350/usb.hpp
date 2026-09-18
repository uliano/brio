/*
 * usb.hpp
 *
 * The RP2350's USB controller in device mode (datasheet 12.7), as the
 * endpoint controller util/usb/device.hpp's stack runs on: `Usb`, a
 * monostate (the chip has one controller and one PHY on its own two
 * pins), realizing the `UsbController` contract at the packet.
 *
 * WHAT THE BLOCK IS. A full-speed PHY on DP and DM with its pull-up, a
 * serial engine each way, and 4 KB of DUAL-PORT RAM (12.7.3.7) that is
 * the whole of the interface: the eight-byte setup packet at offset 0,
 * an endpoint control word per endpoint and direction (the type, the
 * buffer's offset, single or double buffered, the interrupt per
 * buffer), a buffer control word per endpoint and direction (the
 * length, the data PID, FULL, AVAILABLE, STALL, a second half for the
 * second buffer), endpoint zero's 64-byte buffer at 0x100, and 3840
 * bytes of buffers from 0x180 that this driver hands out 64 at a time
 * in the order the stack claims endpoints. The registers at 0x50110000
 * hold the address, the muxing onto the PHY, the SIE control and
 * status (the bus reset, the setup received, suspend and resume), the
 * interrupt quartet and BUFF_STATUS, one bit per endpoint and
 * direction, that says which buffers completed.
 *
 * WHAT IS NOT THE RP2040'S, and every item of it is 12.7.2's:
 *
 *  - MAIN_CTRL.PHY_ISO, THE ONE NEW DUTY. It resets to 1 and isolates
 *    the PHY from the switched core power domain; until it is cleared
 *    the PHY does nothing at all. init() clears it LAST OF ALL - after
 *    the muxing, the power overrides, the controller's own enable, the
 *    SIE and the interrupt mask - which is 12.7.2.2.1's "remove
 *    isolation once software has configured the controller" and leaves
 *    the chapter's register order otherwise exactly as it stands;
 *    release() puts the isolation back, which is the resting state a
 *    power-down wants.
 *  - THREE RESET VALUES MOVED, and all three are answered by writing
 *    the register WHOLE rather than setting bits into it: USB_MUXING
 *    resets with TO_PHY already set, SIE_CTRL with PULLDOWN_EN set
 *    (the host's termination, so that the pins do not float while the
 *    block is unused), MAIN_CTRL with PHY_ISO set.
 *  - DP AND DM CAN BE GPIO HERE. They are pads of GPIO BANK 1 (9.4's
 *    table 647), whose function-select registers are the IO_QSPI
 *    block's USBPHY_DP_CTRL and USBPHY_DM_CTRL and whose SIO face is
 *    the high bank's bits 24 and 25 - GPIO 56 and 57. They must not be
 *    GPIO while this driver owns them, and TWO facts guarantee it: bank
 *    1's FUNCSEL resets to NULL and nothing in this stratum writes it
 *    (rp2350/pin.hpp is bank 0 alone), and init() STORES USB_MUXING
 *    WHOLE, so USBPHY_AS_GPIO, TO_EXTPHY, TO_DIGITAL_PAD and SWAP_DPDM
 *    all land clear whatever a previous image left. No verb of this
 *    file ever sets them.
 *  - ERRATUM RP2350-E12 replaces the RP2040's E16 and is stricter:
 *    eight SIE_STATUS flags and six INTR flags cross from clk_usb to
 *    clk_sys without adequate synchronisation, so CLK_SYS MUST RUN AT
 *    LEAST TEN PER CENT ABOVE CLK_USB while the peripheral is in use.
 *    `usb_clk_sys_ok` is that rule and init() asserts it at compile
 *    time - 52.8 MHz and not 48.
 *  - THE DIAGNOSTICS THE RP2040 HAD NOT: a per-endpoint transmit error
 *    count and a per-endpoint receive error pair (EP_TX_ERROR,
 *    EP_RX_ERROR) that replicate the host's own error count, a short
 *    packet flagged in its own right (SIE_STATUS.RX_SHORT_PACKET), the
 *    two free-running 48 MHz timestamps around the last start of frame,
 *    the state machines exposed in SM_STATE, and DEV_SM_WATCHDOG, a
 *    watchdog that forces the device state machine back to idle if it
 *    hangs. All of them are readbacks and verbs here; none is on the
 *    path a packet takes.
 *  - EVERY RP2040 USB ERRATUM IS FIXED (12.7.2.1), E2 and E5 of the B0
 *    and B1 steppings among them, so nothing of this file is a
 *    workaround.
 *
 * THE BUFFER CONTROL DISCIPLINE (12.7.3.7.1): the word is shared with
 * a controller clocked at 48 MHz while the core runs at 150, so the
 * length, the PID and FULL are written first and AVAILABLE (or STALL)
 * in a second write after a short wait, or the controller could take
 * the new AVAILABLE with the old length. Every submit here does that.
 * This chip also has LINESTATE_TUNING.DEV_BUFF_CONTROL_DOUBLE_READ_FIX,
 * on at reset, which reads the word twice and makes the second write
 * unnecessary - but 12.7.3.7.1 and the warning under table 1194 still
 * prescribe the two-step store, so the two-step store is what this
 * driver does and the fix is a second line of defence. LINESTATE_TUNING
 * is otherwise left exactly as it resets, as 12.7.2 asks: this file
 * READS it and has no verb that writes it.
 *
 * The data PID alternates per endpoint and direction, starts at DATA0
 * after a configure, and endpoint zero's restarts at DATA1 on every
 * setup packet - the stack does not see PIDs, the controller keeps
 * them.
 *
 * ENDPOINT ZERO'S STALL is armed in EP_STALL_ARM as well as in the
 * buffer control word, and the silicon clears both when a setup packet
 * arrives, as chapter 9 wants; the other endpoints' stall is the
 * buffer control's bit alone, cleared by the stack's CLEAR_FEATURE
 * with the PID back at DATA0.
 *
 * THE COMPLETED HALF of a double-buffered OUT endpoint is told from the
 * strict alternation of the halves and each half's own FULL bit, not
 * from BUFF_CPU_SHOULD_HANDLE: the alternation is what the other
 * Raspberry Pi controller was measured to do and this one inherits, and
 * one mechanism proven on silicon is worth more than two. The register
 * is a readback here (`buff_cpu_should_handle`) so that a bench letter
 * can hold the two against each other.
 *
 * THE CLOCK: clk_usb at 48 MHz from the USB PLL, which init() brings up
 * at that rate if it is not already there. `reset run` on this chip
 * does not reset the clock tree, so a PLL already locked on the right
 * ratio is kept and one on a different ratio is re-locked.
 *
 * THE DUAL-PORT RAM TAKES NO ATOMIC ALIASES (12.7.3.7): +0x2000 and
 * +0x3000 are the register block's, not the RAM's, so every write into
 * DPRAM here is a plain store. The REGISTERS take all four, natively
 * (2.1.3), and every status bit of this block - the SIE's flags,
 * BUFF_STATUS, EP_ABORT_DONE, EP_STATUS_STALL_NAK, the error counters,
 * the watchdog's fired bit - is cleared THE SAME ONE WAY, through the
 * CLEAR alias, which is the idiom 12.7.4.2's own listing uses and which
 * leaves whatever else lives in the register alone.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include <span>

#include "rp2350/device.hpp"

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/resets.hpp"
#include "util/usb/device.hpp"

namespace brio {

/// The controller's two regions (the address map, 2.2): the dual-port
/// RAM first, the registers 64 kB above it.
inline constexpr uint32_t usbctrl_dpram_base = USB_DEVICE_DPRAM_BASE;
inline constexpr uint32_t usbctrl_regs_base = USB_BASE;

/**
 * ERRATUM RP2350-E12's rule, as a predicate: clk_sys at least ten per
 * cent above clk_usb while the controller is in use. Under that margin
 * the flags that cross the two domains - TRANS_COMPLETE, SETUP_REC,
 * RX_SHORT_PACKET, ACK_REQ, DATA_SEQ_ERROR, RX_OVERFLOW, STALL_REC,
 * NAK_REC and the six error interrupts - can be lost outright. The
 * quasi-static states (reset, suspend, resume) are exempt, which is
 * why this is the CONTROLLER's rule and not the chip's.
 */
constexpr bool usb_clk_sys_ok(uint32_t sys_hz) {
    return static_cast<uint64_t>(sys_hz) * 10u >= static_cast<uint64_t>(clk_usb_hz) * 11u;
}

/// What the device state-machine watchdog of 12.7.2.2.4 is set to: a
/// limit in clk_usb cycles (18 bits), and whether a fire also forces
/// the state machine to reset rather than only raising the interrupt.
struct UsbDeviceWatchdogConfig {
    uint32_t limit = 0;         ///< 0 .. 0x3FFFF cycles of the 48 MHz domain
    bool reset_on_fire = true;  ///< DEV_SM_WATCHDOG.RESET
};

/// One endpoint's error counters as this chip keeps them (12.7.2.2.4):
/// the transmit count replicates the host's own Cerr, and the two
/// receive flags say which stage of the receive decode complained.
struct UsbEndpointErrors {
    uint8_t tx_count = 0;           ///< EP_TX_ERROR, 0..3
    bool rx_transaction = false;    ///< EP_RX_ERROR's transaction flag
    bool rx_sequence = false;       ///< EP_RX_ERROR's data-sequence flag
    constexpr bool any() const { return tx_count != 0u || rx_transaction || rx_sequence; }
};

struct Usb {
    Usb() = delete;

    static constexpr uint8_t endpoint_count = 16;
    static constexpr uint16_t max_packet = usb_full_speed_packet;
    static constexpr uint32_t dpram_bytes = 4096;
    static constexpr uint32_t ep0_buffer = 0x100;
    static constexpr uint32_t buffers_start = 0x180;
    static constexpr IRQn_Type irq_line = USBCTRL_IRQ_IRQn;

    // ---- the registers ---------------------------------------------------------------
    static USB_Type& regs() { return *USB; }
    static volatile uint32_t& dpram_word(uint32_t offset) {
        return *reinterpret_cast<volatile uint32_t*>(static_cast<uintptr_t>(usbctrl_dpram_base + offset));
    }
    static volatile uint8_t* dpram_bytes_at(uint32_t offset) {
        return reinterpret_cast<volatile uint8_t*>(static_cast<uintptr_t>(usbctrl_dpram_base + offset));
    }
    /// The endpoint control word of endpoint 1..15, one direction
    /// (table 1192; endpoint zero has none, its buffers being fixed).
    static volatile uint32_t& ep_control(uint8_t number, bool in) {
        return dpram_word(0x08u + 8u * (number - 1u) + (in ? 0u : 4u));
    }
    /// The buffer control word of endpoint 0..15, one direction.
    static volatile uint32_t& buf_control(uint8_t number, bool in) {
        return dpram_word(0x80u + 8u * number + (in ? 0u : 4u));
    }

    /**
     * clk_usb from the USB PLL at 48 MHz (brought up if it is not
     * already at that rate), the block out of reset, its RAM cleared,
     * the PHY muxed in and taken off the GPIO bank, VBUS detection
     * forced (the boards wire no VBUS pin), the host's pull-downs
     * dropped, device mode, THE PHY'S ISOLATION LIFTED, and the
     * interrupts of a device (a buffer done, a bus reset, a setup
     * packet, suspend, resume) enabled at the block and at this core's
     * interrupt controller. The pull-up stays down: the stack raises it
     * when it is ready to answer.
     */
    template <typename Clock>
    static bool init(Clock) {
        static_assert(usb_clk_sys_ok(Clock::hz),
                      "brio Usb: erratum RP2350-E12 - clk_sys must run at least 10 per cent "
                      "above clk_usb (48 MHz), so above 52.8 MHz, while the controller is in use");
        constexpr PllConfig cfg = pll_config_for(Clock::xtal_hz, clk_usb_hz);
        static_assert(cfg.valid(), "brio Usb: no exact USB PLL ratio to 48 MHz from this crystal");
        if (!PllUsb::locked() || !(PllUsb::config() == cfg)) {
            if (!PllUsb::init(cfg)) {
                return false;
            }
        }
        if (!Clocks::usb_select(UsbAux::pll_usb, 1)) {
            return false;
        }
        if (!Resets::cycle(ResetBlock::usbctrl)) {
            return false;
        }
        for (uint32_t off = 0; off < dpram_bytes; off += 4u) {
            dpram_word(off) = 0;
        }
        next_buffer_ = buffers_start;
        for (uint8_t n = 0; n < endpoint_count; ++n) {
            pid_in_[n] = 0;
            pid_out_[n] = 0;
            out_length_[n] = 0;
            out_buffer_[n] = 0;
            in_buffer_[n] = 0;
            out_twin_[n] = false;
            out_fresh_[n] = false;
            out_arm_half_[n] = 0;
            out_done_half_[n] = 0;
            out_data_half_[n] = 0;
        }
        out_buffer_[0] = ep0_buffer;
        in_buffer_[0] = ep0_buffer;
        // Written WHOLE, not set into: USBPHY_AS_GPIO, TO_EXTPHY,
        // TO_DIGITAL_PAD and SWAP_DPDM land clear here and nothing in
        // this file ever raises them.
        regs().USB_MUXING = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
        regs().USB_PWR = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
        // The controller enabled in device mode WITH THE PHY STILL
        // ISOLATED: the order of the registers is the chapter's own, and
        // the isolation comes off at the end of it.
        regs().MAIN_CTRL = USB_MAIN_CTRL_CONTROLLER_EN_BITS | USB_MAIN_CTRL_PHY_ISO_BITS;
        // Written WHOLE again: PULLDOWN_EN resets SET on this chip and
        // a device must not pull its own lines down.
        regs().SIE_CTRL = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
        regs().INTE = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS |
                      USB_INTS_DEV_SUSPEND_BITS | USB_INTS_DEV_RESUME_FROM_HOST_BITS;
        // THE ONE NEW DUTY, and last of all: "remove isolation once
        // software has configured the controller" (12.7.2.2.1).
        phy_isolate(false);
        Irq::enable(irq_line);
        return true;
    }

    /// The line released, the PHY isolated again (the resting state a
    /// power-down wants), the block back into reset. Its first three
    /// acts are register writes, so it is for a block that init() has
    /// taken out of reset and not for a cold one - a peripheral held in
    /// reset answers the bus with an error (7.5).
    static void release() {
        Irq::disable(irq_line);
        connect(false);
        regs().INTE = 0;
        phy_isolate(true);
        Resets::hold(ResetBlock::usbctrl);
    }

    // ---- the UsbController contract ---------------------------------------------------
    static void connect(bool on) {
        if (on) {
            hw_set(regs().SIE_CTRL, USB_SIE_CTRL_PULLUP_EN_BITS);
        } else {
            hw_clear(regs().SIE_CTRL, USB_SIE_CTRL_PULLUP_EN_BITS);
        }
    }

    static void set_address(uint8_t address) { regs().ADDR_ENDP = address & USB_ADDR_ENDP_ADDRESS_BITS; }

    /// A bulk OUT endpoint is DOUBLE BUFFERED (two 64-byte halves, the
    /// controller flipping between them): a packet lands in one while
    /// the stack empties the other, so the host is never NAKed between
    /// packets - and a NAK costs a whole frame when the device sits
    /// behind a hub's transaction translator. Every other endpoint is
    /// single buffered.
    static bool configure_endpoint(uint8_t address, UsbEndpointType type, uint16_t max) {
        const uint8_t n = usb_ep_number(address);
        const bool in = usb_ep_is_in(address);
        const bool twin = !in && type == UsbEndpointType::bulk;
        const uint32_t bytes = twin ? 128u : 64u;
        if (n == 0u || n >= endpoint_count || max > max_packet || type == UsbEndpointType::control ||
            next_buffer_ + bytes > dpram_bytes) {
            return false;
        }
        const uint32_t buffer = next_buffer_;
        next_buffer_ += bytes;
        (in ? in_buffer_ : out_buffer_)[n] = buffer;
        (in ? pid_in_ : pid_out_)[n] = 0;
        if (!in) {   // the OUT direction's state alone: the IN of the same number is another endpoint
            out_twin_[n] = twin;
            out_arm_half_[n] = 0;
            out_done_half_[n] = 0;
            out_data_half_[n] = 0;
            out_fresh_[n] = twin;
        }
        buf_control(n, in) = 0;
        ep_control(n, in) =
            USB_DEVICE_DPRAM_EP1_IN_CONTROL_ENABLE_BITS | USB_DEVICE_DPRAM_EP1_IN_CONTROL_INTERRUPT_PER_BUFF_BITS |
            (twin ? USB_DEVICE_DPRAM_EP1_IN_CONTROL_DOUBLE_BUFFERED_BITS : 0u) |
            (static_cast<uint32_t>(type) << USB_DEVICE_DPRAM_EP1_IN_CONTROL_ENDPOINT_TYPE_LSB) | buffer;
        return true;
    }

    /// How many OUT packets may be armed at once on a bulk endpoint
    /// (the two halves); a class arms that many while its ring has room.
    static constexpr uint8_t out_slots = 2;

    static void deconfigure_endpoints() {
        for (uint8_t n = 1; n < endpoint_count; ++n) {
            ep_control(n, true) = 0;
            ep_control(n, false) = 0;
            buf_control(n, true) = 0;
            buf_control(n, false) = 0;
            in_buffer_[n] = 0;
            out_buffer_[n] = 0;
            out_twin_[n] = false;
        }
        next_buffer_ = buffers_start;
    }

    static void stall(uint8_t address, bool on) {
        const uint8_t n = usb_ep_number(address);
        const bool in = usb_ep_is_in(address);
        if (n == 0u) {
            const uint32_t arm = in ? USB_EP_STALL_ARM_EP0_IN_BITS : USB_EP_STALL_ARM_EP0_OUT_BITS;
            if (on) {
                hw_set(regs().EP_STALL_ARM, arm);
            } else {
                hw_clear(regs().EP_STALL_ARM, arm);
            }
        }
        if (on) {
            buf_control(n, in) = USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_STALL_BITS;
        } else {
            buf_control(n, in) = 0;
            (in ? pid_in_ : pid_out_)[n] = 0;
        }
    }

    static bool stalled(uint8_t address) {
        return (buf_control(usb_ep_number(address), usb_ep_is_in(address)) &
                USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_STALL_BITS) != 0u;
    }

    /// One packet copied into the endpoint's buffer and offered: the
    /// controller answers the next IN token with it.
    static bool submit_in(uint8_t number, std::span<const uint8_t> data) {
        if (number >= endpoint_count || data.size() > max_packet || in_buffer_[number] == 0u) {
            return false;
        }
        volatile uint8_t* dst = dpram_bytes_at(in_buffer_[number]);
        for (size_t i = 0; i < data.size(); ++i) {
            dst[i] = data[i];
        }
        uint32_t word = static_cast<uint32_t>(data.size()) | USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_FULL_0_BITS;
        if (pid_in_[number] != 0u) {
            word |= USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_PID_0_BITS;
        }
        pid_in_[number] ^= 1u;
        offer(buf_control(number, true), word);
        return true;
    }

    /// The endpoint's buffer offered for one packet of at most `max`: on
    /// a double-buffered endpoint the next half in turn, its 16-bit
    /// control written alone so the other half's stays as it is.
    static bool submit_out(uint8_t number, uint16_t max) {
        if (number >= endpoint_count || max > max_packet || out_buffer_[number] == 0u) {
            return false;
        }
        uint16_t word = max;
        if (pid_out_[number] != 0u) {
            word |= static_cast<uint16_t>(USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_PID_0_BITS);
        }
        pid_out_[number] ^= 1u;
        if (out_twin_[number]) {
            // One half armed by its own 16-bit control, the other's left
            // to the controller (a whole-word write could re-arm a half
            // that completed a moment ago).
            const uint8_t half = out_arm_half_[number];
            out_arm_half_[number] ^= 1u;
            if (out_fresh_[number]) {
                // The controller's buffer select is internal and keeps its
                // last value across a reconfigure: the first offer after
                // one puts it back on buffer 0 (RESET_BUFSEL, cleared by
                // the transfer).
                out_fresh_[number] = false;
                word |= static_cast<uint16_t>(USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_RESET_BITS);
            }
            offer_half(buf_control_half(number, false, half), word);
        } else {
            offer(buf_control(number, false), word);
        }
        return true;
    }

    /// The last packet that came on the endpoint: the bytes in the
    /// controller's RAM (the half that completed), valid until the
    /// next submit_out lands there.
    static std::span<const uint8_t> out_data(uint8_t number) {
        if (number >= endpoint_count || out_buffer_[number] == 0u) {
            return {};
        }
        const uint32_t at = out_buffer_[number] + (out_twin_[number] && out_data_half_[number] != 0u ? 64u : 0u);
        return {const_cast<const uint8_t*>(dpram_bytes_at(at)), out_length_[number]};
    }

    static UsbSetup setup() {
        const uint32_t low = dpram_word(0);
        const uint32_t high = dpram_word(4);
        return UsbSetup{.request_type = static_cast<uint8_t>(low),
                        .request = static_cast<uint8_t>(low >> 8),
                        .value = static_cast<uint16_t>(low >> 16),
                        .index = static_cast<uint16_t>(high),
                        .length = static_cast<uint16_t>(high >> 16)};
    }

    /// What happened since the last call, acknowledged: the SIE flags
    /// cleared, BUFF_STATUS cleared bit by bit with the length of each
    /// completed OUT packet read first. A setup packet restarts
    /// endpoint zero's PIDs at DATA1 both ways.
    static UsbEvents take_events() {
        UsbEvents ev{};
        const uint32_t status = regs().INTS;
        if ((status & USB_INTS_BUS_RESET_BITS) != 0u) {
            hw_clear(regs().SIE_STATUS, USB_SIE_STATUS_BUS_RESET_BITS);
            ev.reset = true;
        }
        if ((status & USB_INTS_SETUP_REQ_BITS) != 0u) {
            hw_clear(regs().SIE_STATUS, USB_SIE_STATUS_SETUP_REC_BITS);
            pid_in_[0] = 1;
            pid_out_[0] = 1;
            ev.setup = true;
        }
        if ((status & USB_INTS_DEV_SUSPEND_BITS) != 0u) {
            hw_clear(regs().SIE_STATUS, USB_SIE_STATUS_SUSPENDED_BITS);
            ev.suspend = true;
        }
        if ((status & USB_INTS_DEV_RESUME_FROM_HOST_BITS) != 0u) {
            hw_clear(regs().SIE_STATUS, USB_SIE_STATUS_RESUME_BITS);
            ev.resume = true;
        }
        if ((status & USB_INTS_BUFF_STATUS_BITS) != 0u) {
            const uint32_t done = regs().BUFF_STATUS;
            for (uint8_t n = 0; n < endpoint_count; ++n) {
                const uint32_t in_bit = 1u << (2u * n);
                const uint32_t out_bit = 1u << (2u * n + 1u);
                if ((done & in_bit) != 0u) {
                    ev.in_done |= static_cast<uint16_t>(1u << n);
                }
                if ((done & out_bit) != 0u) {
                    const uint32_t word = buf_control(n, false);
                    if (out_twin_[n]) {
                        // The halves complete in strict alternation, like the
                        // controller's own select: the one expected is the one
                        // handled, its FULL bit the proof - when both halves
                        // completed before this ran, the status bit re-sets
                        // itself for the second and a pass finds it; a pass
                        // that finds the expected half not full has nothing
                        // to report.
                        const uint8_t half = out_done_half_[n];
                        const uint32_t mine = half == 0u ? word : (word >> 16);
                        if ((mine & USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_FULL_0_BITS) != 0u) {
                            out_data_half_[n] = half;
                            out_done_half_[n] = half ^ 1u;
                            out_length_[n] =
                                static_cast<uint16_t>(mine & USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_LENGTH_0_BITS);
                            ev.out_done |= static_cast<uint16_t>(1u << n);
                        }
                    } else {
                        out_length_[n] =
                            static_cast<uint16_t>(word & USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_LENGTH_0_BITS);
                        ev.out_done |= static_cast<uint16_t>(1u << n);
                    }
                }
            }
            hw_clear(regs().BUFF_STATUS, done);
        }
        return ev;
    }

    // ---- readbacks ---------------------------------------------------------------------
    static bool connected() { return (regs().SIE_STATUS & USB_SIE_STATUS_CONNECTED_BITS) != 0u; }
    static bool suspended() { return (regs().SIE_STATUS & USB_SIE_STATUS_SUSPENDED_BITS) != 0u; }
    static bool vbus_detected() { return (regs().SIE_STATUS & USB_SIE_STATUS_VBUS_DETECTED_BITS) != 0u; }
    static bool pulled_up() { return (regs().SIE_CTRL & USB_SIE_CTRL_PULLUP_EN_BITS) != 0u; }
    static uint8_t address() { return static_cast<uint8_t>(regs().ADDR_ENDP & USB_ADDR_ENDP_ADDRESS_BITS); }
    /// The last start-of-frame number seen (a frame a millisecond while the host is awake).
    static uint16_t frame() { return static_cast<uint16_t>(regs().SOF_RD & USB_SOF_RD_COUNT_BITS); }
    static uint32_t sie_status() { return regs().SIE_STATUS; }
    /// SIE_STATUS.LINE_STATE, the two-bit code the line state detector
    /// reports. The chapter names the field and gives no key to its
    /// values, so this hands the code back as it stands.
    static uint8_t line_state() {
        return static_cast<uint8_t>((regs().SIE_STATUS & USB_SIE_STATUS_LINE_STATE_BITS) >>
                                    USB_SIE_STATUS_LINE_STATE_LSB);
    }
    /// The error flags of the SIE (CRC, bit stuff, overflow, timeout, data sequence), read and cleared.
    static uint32_t take_errors() {
        constexpr uint32_t errors = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS | USB_SIE_STATUS_RX_TIMEOUT_BITS |
                                    USB_SIE_STATUS_RX_OVERFLOW_BITS | USB_SIE_STATUS_BIT_STUFF_ERROR_BITS |
                                    USB_SIE_STATUS_CRC_ERROR_BITS;
        const uint32_t seen = regs().SIE_STATUS & errors;
        if (seen != 0u) {
            hw_clear(regs().SIE_STATUS, seen);
        }
        return seen;
    }
    static uint32_t buffers_used() { return next_buffer_; }

    // ---- the PHY's isolation, this chip's one new duty ---------------------------------
    /// MAIN_CTRL.PHY_ISO: set at reset and after a power down of the
    /// switched core domain, and the PHY answers nothing while it
    /// stands. init() clears it; release() puts it back.
    static bool phy_isolated() { return (regs().MAIN_CTRL & USB_MAIN_CTRL_PHY_ISO_BITS) != 0u; }
    static void phy_isolate(bool on) {
        if (on) {
            hw_set(regs().MAIN_CTRL, USB_MAIN_CTRL_PHY_ISO_BITS);
        } else {
            hw_clear(regs().MAIN_CTRL, USB_MAIN_CTRL_PHY_ISO_BITS);
        }
    }
    static bool controller_enabled() { return (regs().MAIN_CTRL & USB_MAIN_CTRL_CONTROLLER_EN_BITS) != 0u; }
    /// Whether the muxing hands DP and DM to the SIO's high bank as
    /// ordinary pads instead of to the controller. Always false while
    /// this driver owns them: nothing here sets the bit.
    static bool phy_as_gpio() { return (regs().USB_MUXING & USB_USB_MUXING_USBPHY_AS_GPIO_BITS) != 0u; }
    static uint32_t muxing() { return regs().USB_MUXING; }

    // ---- LINESTATE_TUNING, read and never written --------------------------------------
    /// The register 12.7.2 says to leave alone. These are readbacks of
    /// the fixes that are on by DEFAULT, so that a program can report
    /// what the silicon is doing rather than change it.
    static uint32_t linestate_tuning() { return regs().LINESTATE_TUNING; }
    static bool buffer_control_double_read_fix() {
        return (regs().LINESTATE_TUNING & USB_LINESTATE_TUNING_DEV_BUFF_CONTROL_DOUBLE_READ_FIX_BITS) != 0u;
    }
    static bool wake_on_any_bus_activity() {
        return (regs().LINESTATE_TUNING & USB_LINESTATE_TUNING_DEV_LS_WAKE_FIX_BITS) != 0u;
    }

    // ---- the diagnostics this chip added (12.7.2.2.4) ----------------------------------
    /// One endpoint's error counters, the transmit count and the two
    /// receive flags. Three consecutive errors is where the host gives
    /// up on an endpoint, which is what makes the count worth reading.
    static UsbEndpointErrors endpoint_errors(uint8_t number) {
        if (number >= endpoint_count) {
            return {};
        }
        const uint32_t tx = regs().EP_TX_ERROR;
        const uint32_t rx = regs().EP_RX_ERROR;
        return UsbEndpointErrors{
            .tx_count = static_cast<uint8_t>((tx >> (2u * number)) & 0x3u),
            .rx_transaction = (rx & (1u << (2u * number))) != 0u,
            .rx_sequence = (rx & (1u << (2u * number + 1u))) != 0u,
        };
    }
    /// Every endpoint's counters back to zero, both registers in one
    /// store each through the CLEAR alias.
    static void clear_endpoint_errors() {
        hw_clear(regs().EP_TX_ERROR, 0xFFFF'FFFFu);
        hw_clear(regs().EP_RX_ERROR, 0xFFFF'FFFFu);
    }
    /// SIE_STATUS.ENDPOINT_ERROR: at least one endpoint's counters are
    /// not zero. Read and cleared.
    static bool take_endpoint_error() {
        const bool seen = (regs().SIE_STATUS & USB_SIE_STATUS_ENDPOINT_ERROR_BITS) != 0u;
        if (seen) {
            hw_clear(regs().SIE_STATUS, USB_SIE_STATUS_ENDPOINT_ERROR_BITS);
        }
        return seen;
    }
    /// SIE_STATUS.RX_SHORT_PACKET: the last OUT was shorter than the
    /// buffer offered. The stack reads the length instead; this is the
    /// flag beside it. Read and cleared.
    static bool take_short_packet() {
        const bool seen = (regs().SIE_STATUS & USB_SIE_STATUS_RX_SHORT_PACKET_BITS) != 0u;
        if (seen) {
            hw_clear(regs().SIE_STATUS, USB_SIE_STATUS_RX_SHORT_PACKET_BITS);
        }
        return seen;
    }
    /// SIE_CTRL.EP0_STOP_ON_SHORT_PACKET: a short packet on endpoint
    /// zero stops the transaction instead of running on. Off here - the
    /// control machine of util/usb/device.hpp ends its own data stage on
    /// a short packet - and a verb because the chapter has it.
    static void ep0_stop_on_short_packet(bool on) {
        if (on) {
            hw_set(regs().SIE_CTRL, USB_SIE_CTRL_EP0_STOP_ON_SHORT_PACKET_BITS);
        } else {
            hw_clear(regs().SIE_CTRL, USB_SIE_CTRL_EP0_STOP_ON_SHORT_PACKET_BITS);
        }
    }

    /// The free-running 21-bit counter of the 48 MHz domain, and its
    /// value at the last start of frame: their difference is how long
    /// ago the host's last frame was, in 48ths of a microsecond.
    static uint32_t sof_timestamp_raw() { return regs().SOF_TIMESTAMP_RAW & USB_SOF_TIMESTAMP_RAW_BITS; }
    static uint32_t sof_timestamp_last() { return regs().SOF_TIMESTAMP_LAST & USB_SOF_TIMESTAMP_LAST_BITS; }
    /// clk_usb cycles since the last start of frame, the 21-bit wrap
    /// taken into account. About 48 000 between frames on a live bus.
    static uint32_t since_sof() {
        constexpr uint32_t span = USB_SOF_TIMESTAMP_RAW_BITS + 1u;
        return (sof_timestamp_raw() + span - sof_timestamp_last()) % span;
    }

    /// SM_STATE (12.7.2.2.1): the controller's own state machines, for
    /// a program that wants to say where a stuck controller is stuck.
    static uint32_t sm_state() { return regs().SM_STATE & USB_SM_STATE_BITS; }
    static uint8_t sm_main() {
        return static_cast<uint8_t>((regs().SM_STATE & USB_SM_STATE_STATE_BITS) >> USB_SM_STATE_STATE_LSB);
    }
    static uint8_t sm_bus_control() {
        return static_cast<uint8_t>((regs().SM_STATE & USB_SM_STATE_BC_STATE_BITS) >> USB_SM_STATE_BC_STATE_LSB);
    }
    static uint8_t sm_rx_deserialiser() {
        return static_cast<uint8_t>((regs().SM_STATE & USB_SM_STATE_RX_DASM_BITS) >> USB_SM_STATE_RX_DASM_LSB);
    }

    /// The device state machine's watchdog: a limit in clk_usb cycles
    /// and, when it fires, the machine forced back to idle. The limit is
    /// written with the watchdog DISABLED, as the register description
    /// requires, and the enable follows in its own store.
    static bool arm_device_watchdog(const UsbDeviceWatchdogConfig& cfg) {
        if (cfg.limit > USB_DEV_SM_WATCHDOG_LIMIT_BITS) {
            return false;
        }
        regs().DEV_SM_WATCHDOG = cfg.limit | (cfg.reset_on_fire ? USB_DEV_SM_WATCHDOG_RESET_BITS : 0u);
        hw_set(regs().DEV_SM_WATCHDOG, USB_DEV_SM_WATCHDOG_ENABLE_BITS);
        return true;
    }
    static void disarm_device_watchdog() { hw_clear(regs().DEV_SM_WATCHDOG, USB_DEV_SM_WATCHDOG_ENABLE_BITS); }
    static bool device_watchdog_armed() {
        return (regs().DEV_SM_WATCHDOG & USB_DEV_SM_WATCHDOG_ENABLE_BITS) != 0u;
    }
    static uint32_t device_watchdog_limit() { return regs().DEV_SM_WATCHDOG & USB_DEV_SM_WATCHDOG_LIMIT_BITS; }
    /// Whether the watchdog fired since the last look. The flag is
    /// cleared through the CLEAR alias, like every other status bit of
    /// this block, which leaves the three settings beside it alone.
    static bool take_device_watchdog_fired() {
        const bool seen = (regs().DEV_SM_WATCHDOG & USB_DEV_SM_WATCHDOG_FIRED_BITS) != 0u;
        if (seen) {
            hw_clear(regs().DEV_SM_WATCHDOG, USB_DEV_SM_WATCHDOG_FIRED_BITS);
        }
        return seen;
    }

    // ---- the abort path, which E2 made useless on the other chip ------------------------
    /// The endpoint's buffer control ignored and every access NAKed
    /// until the abort is lifted: the way to revoke a buffer the host
    /// has not taken. `abort_done` says when the endpoint is idle and
    /// the buffer control may be written.
    static void abort(uint8_t address, bool on) {
        const uint32_t bit = endpoint_bit(address);
        if (on) {
            hw_set(regs().EP_ABORT, bit);
        } else {
            hw_clear(regs().EP_ABORT, bit);
            hw_clear(regs().EP_ABORT_DONE, bit);   // a status bit, cleared like the others
        }
    }
    static bool aborted(uint8_t address) { return (regs().EP_ABORT & endpoint_bit(address)) != 0u; }
    static bool abort_done(uint8_t address) { return (regs().EP_ABORT_DONE & endpoint_bit(address)) != 0u; }

    // ---- the stall and NAK reporting of an endpoint -------------------------------------
    /// EP_STATUS_STALL_NAK: the bits an endpoint's INTERRUPT_ON_STALL
    /// and INTERRUPT_ON_NAK raise (endpoint zero's come from SIE_CTRL).
    static uint32_t stall_nak_status() { return regs().EP_STATUS_STALL_NAK; }
    static void clear_stall_nak(uint32_t mask) { hw_clear(regs().EP_STATUS_STALL_NAK, mask); }
    /// Endpoint zero's two reporting bits, which live in SIE_CTRL and
    /// not in an endpoint control word - endpoint zero has none.
    static void ep0_report(bool on_stall, bool on_nak) {
        hw_write_masked(regs().SIE_CTRL,
                        (on_stall ? USB_SIE_CTRL_EP0_INT_STALL_BITS : 0u) |
                            (on_nak ? USB_SIE_CTRL_EP0_INT_NAK_BITS : 0u),
                        USB_SIE_CTRL_EP0_INT_STALL_BITS | USB_SIE_CTRL_EP0_INT_NAK_BITS);
    }

    /// BUFF_CPU_SHOULD_HANDLE, the register that NAMES the half of a
    /// double-buffered endpoint the controller finished with. Not on
    /// the path a packet takes here (the file header says why), and a
    /// readback so that the two answers can be held against each other.
    static uint32_t buff_cpu_should_handle() { return regs().BUFF_CPU_SHOULD_HANDLE; }
    static uint32_t buff_status() { return regs().BUFF_STATUS; }

    // ---- the remote wakeup ---------------------------------------------------------------
    /// SIE_CTRL.RESUME: the device asking a suspended host to wake it.
    /// Legal only when the host has set DEVICE_REMOTE_WAKEUP, which the
    /// stack takes and does not act on - this is the verb that would.
    static void remote_wakeup() { hw_set(regs().SIE_CTRL, USB_SIE_CTRL_RESUME_BITS); }

    // ---- the interrupt surface -------------------------------------------------------------
    static uint32_t interrupt_enable() { return regs().INTE; }
    static void interrupt_enable(uint32_t mask) { regs().INTE = mask; }
    static uint32_t interrupt_raw() { return regs().INTR; }
    static uint32_t interrupt_status() { return regs().INTS; }
    static void force_interrupt(uint32_t mask) { regs().INTF = mask; }
    static uint32_t forced_interrupts() { return regs().INTF; }
    static constexpr IRQn_Type irq() { return irq_line; }

private:
    /// The two-step write of 12.7.3.7.1: the word without AVAILABLE, a
    /// few clk_sys cycles so that at least one clk_usb cycle passes,
    /// then AVAILABLE. `nop` is a mnemonic both of this chip's
    /// instruction sets have, which is why the spin is written in the
    /// assembler and not in CMSIS.
    static void offer(volatile uint32_t& control, uint32_t word) {
        control = word;
        for (uint8_t spins = 0; spins < 12u; ++spins) {
            __asm__ volatile("nop");
        }
        control = word | USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_AVAILABLE_0_BITS;
    }

    /// One half of a double-buffered endpoint's control word (the RAM
    /// takes 8-, 16- and 32-bit accesses, 12.7.3.7).
    static volatile uint16_t& buf_control_half(uint8_t number, bool in, uint8_t half) {
        return *reinterpret_cast<volatile uint16_t*>(
            static_cast<uintptr_t>(usbctrl_dpram_base + 0x80u + 8u * number + (in ? 0u : 4u) + 2u * half));
    }
    static void offer_half(volatile uint16_t& control, uint16_t word) {
        control = word;
        for (uint8_t spins = 0; spins < 12u; ++spins) {
            __asm__ volatile("nop");
        }
        control = static_cast<uint16_t>(word | USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_AVAILABLE_0_BITS);
    }

    /// The bit an endpoint and direction occupy in the four registers
    /// that are laid out that way: BUFF_STATUS, BUFF_CPU_SHOULD_HANDLE,
    /// EP_ABORT and EP_STATUS_STALL_NAK. IN is the even bit.
    static uint32_t endpoint_bit(uint8_t address) {
        const uint32_t n = usb_ep_number(address);
        return 1u << (2u * n + (usb_ep_is_in(address) ? 0u : 1u));
    }

    static inline uint32_t next_buffer_ = buffers_start;
    static inline bool out_twin_[endpoint_count]{};
    static inline bool out_fresh_[endpoint_count]{};
    static inline uint8_t out_arm_half_[endpoint_count]{};
    static inline uint8_t out_done_half_[endpoint_count]{};
    static inline uint8_t out_data_half_[endpoint_count]{};
    static inline uint32_t in_buffer_[endpoint_count]{};
    static inline uint32_t out_buffer_[endpoint_count]{};
    static inline uint16_t out_length_[endpoint_count]{};
    static inline uint8_t pid_in_[endpoint_count]{};
    static inline uint8_t pid_out_[endpoint_count]{};
};

static_assert(UsbController<Usb>);

} // namespace brio
