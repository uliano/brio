/*
 * usb.hpp
 *
 * The RP2040's USB controller in device mode (datasheet 4.1), as the
 * endpoint controller util/usb/device.hpp's stack runs on: `Usb`, a
 * monostate (the chip has one controller and one PHY on its own two
 * pins), realizing the `UsbController` contract at the packet.
 *
 * WHAT THE BLOCK IS. A full-speed PHY on DP/DM with its pull-up, a
 * serial engine each way, and 4 KB of DUAL-PORT RAM (4.1.2.7) that is
 * the whole of the interface: the eight-byte setup packet at offset 0,
 * an endpoint control word per endpoint and direction (the type, the
 * buffer's offset, the interrupt per buffer), a buffer control word
 * per endpoint and direction (the length, the data PID, FULL,
 * AVAILABLE, STALL), endpoint zero's 64-byte buffer at 0x100, and
 * 3840 bytes of buffers from 0x180 that this driver hands out 64 at a
 * time in the order the stack claims endpoints. The registers at
 * 0x50110000 hold the address, the muxing onto the PHY, the SIE
 * control and status (the bus reset, the setup received, suspend and
 * resume), the interrupt trio and BUFF_STATUS, one bit per endpoint
 * and direction, that says which buffers completed.
 *
 * THE BUFFER CONTROL DISCIPLINE (4.1.2.7.1): the word is shared with
 * a controller clocked at 48 MHz while the core runs at 125, so the
 * length, the PID and FULL are written first and AVAILABLE (or STALL)
 * in a second write after a short wait, or the controller could take
 * the new AVAILABLE with the old length. Every submit here does that.
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
 * THE CLOCK: clk_usb at 48 MHz from the USB PLL, which init() brings
 * up at that rate if it is not already (the converter shares it,
 * rp2040/adc.hpp), and clk_sys above 48 MHz (erratum E16).
 *
 * The two errata of the device controller (E2, E5) are fixed on the
 * B2 silicon of every board on the bench; this driver carries no
 * workaround for the B0 and B1 steppings.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <span>

#include "rp2040/clock.hpp"
#include "rp2040/device.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/resets.hpp"
#include "util/usb/device.hpp"

namespace brio {

/// The controller's two regions (the address map, 2.2).
inline constexpr uint32_t usbctrl_dpram_base = 0x5010'0000u;
inline constexpr uint32_t usbctrl_regs_base = 0x5011'0000u;
inline constexpr uint32_t usb_clk_hz = 48'000'000;

struct Usb {
    Usb() = delete;

    static constexpr uint8_t endpoint_count = 16;
    static constexpr uint16_t max_packet = usb_full_speed_packet;
    static constexpr uint32_t dpram_bytes = 4096;
    static constexpr uint32_t ep0_buffer = 0x100;
    static constexpr uint32_t buffers_start = 0x180;
    static constexpr IRQn_Type irq_line = USBCTRL_IRQ_IRQn;

    // ---- the registers ---------------------------------------------------------------
    static volatile uint32_t& reg(uint32_t offset) { return reg_at(usbctrl_regs_base, offset); }
    static volatile uint32_t& dpram_word(uint32_t offset) {
        return *reinterpret_cast<volatile uint32_t*>(static_cast<uintptr_t>(usbctrl_dpram_base + offset));
    }
    static volatile uint8_t* dpram_bytes_at(uint32_t offset) {
        return reinterpret_cast<volatile uint8_t*>(static_cast<uintptr_t>(usbctrl_dpram_base + offset));
    }
    /// The endpoint control word of endpoint 1..15, one direction.
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
     * the PHY muxed in, VBUS detection forced (the boards do not wire
     * VBUS to a pin), device mode, the interrupts of a device (a
     * buffer done, a bus reset, a setup packet, suspend, resume)
     * enabled at the block and at this core's NVIC. The pull-up stays
     * down: the stack raises it when it is ready to answer.
     */
    template <typename Clock>
    static bool init(Clock) {
        static_assert(Clock::hz > usb_clk_hz, "brio Usb: clk_sys must run above 48 MHz (erratum E16)");
        constexpr PllConfig cfg = pll_config_for(Clock::xtal_hz, usb_clk_hz);
        static_assert(cfg.valid(), "brio Usb: no exact USB PLL ratio to 48 MHz from this crystal");
        if (!PllUsb::locked() || !(PllUsb::config() == cfg)) {
            if (!PllUsb::init(cfg)) {
                return false;
            }
        }
        Clocks::usb_select(UsbAux::pll_usb, 1);
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
            out_arm_half_[n] = 0;
            out_done_half_[n] = 0;
            out_data_half_[n] = 0;
        }
        out_buffer_[0] = ep0_buffer;
        in_buffer_[0] = ep0_buffer;
        reg(USB_USB_MUXING_OFFSET) = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
        reg(USB_USB_PWR_OFFSET) = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
        reg(USB_MAIN_CTRL_OFFSET) = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
        reg(USB_SIE_CTRL_OFFSET) = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
        reg(USB_INTE_OFFSET) = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS |
                               USB_INTS_DEV_SUSPEND_BITS | USB_INTS_DEV_RESUME_FROM_HOST_BITS;
        Nvic::enable(irq_line);
        return true;
    }

    /// The block back into reset, the line released.
    static void release() {
        Nvic::disable(irq_line);
        connect(false);
        reg(USB_INTE_OFFSET) = 0;
        Resets::hold(ResetBlock::usbctrl);
    }

    // ---- the UsbController contract ---------------------------------------------------
    static void connect(bool on) {
        if (on) { hw_set(reg(USB_SIE_CTRL_OFFSET), USB_SIE_CTRL_PULLUP_EN_BITS); } else { hw_clear(reg(USB_SIE_CTRL_OFFSET), USB_SIE_CTRL_PULLUP_EN_BITS); }
    }

    static void set_address(uint8_t address) { reg(USB_ADDR_ENDP_OFFSET) = address & 0x7Fu; }

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
        ep_control(n, in) = (1u << 31) | (1u << 29) | (twin ? (1u << 30) : 0u) | (static_cast<uint32_t>(type) << 26) | buffer;
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
            if (on) { hw_set(reg(USB_EP_STALL_ARM_OFFSET), arm); } else { hw_clear(reg(USB_EP_STALL_ARM_OFFSET), arm); }
        }
        if (on) {
            buf_control(n, in) = USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_STALL_BITS;
        } else {
            buf_control(n, in) = 0;
            (in ? pid_in_ : pid_out_)[n] = 0;
        }
    }

    static bool stalled(uint8_t address) {
        return (buf_control(usb_ep_number(address), usb_ep_is_in(address)) & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_STALL_BITS) != 0u;
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
                word |= static_cast<uint16_t>(1u << 12);
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
        const uint32_t status = reg(USB_INTS_OFFSET);
        if ((status & USB_INTS_BUS_RESET_BITS) != 0u) {
            hw_clear(reg(USB_SIE_STATUS_OFFSET), USB_SIE_STATUS_BUS_RESET_BITS);
            ev.reset = true;
        }
        if ((status & USB_INTS_SETUP_REQ_BITS) != 0u) {
            hw_clear(reg(USB_SIE_STATUS_OFFSET), USB_SIE_STATUS_SETUP_REC_BITS);
            pid_in_[0] = 1;
            pid_out_[0] = 1;
            ev.setup = true;
        }
        if ((status & USB_INTS_DEV_SUSPEND_BITS) != 0u) {
            hw_clear(reg(USB_SIE_STATUS_OFFSET), USB_SIE_STATUS_SUSPENDED_BITS);
            ev.suspend = true;
        }
        if ((status & USB_INTS_DEV_RESUME_FROM_HOST_BITS) != 0u) {
            hw_clear(reg(USB_SIE_STATUS_OFFSET), USB_SIE_STATUS_RESUME_BITS);
            ev.resume = true;
        }
        if ((status & USB_INTS_BUFF_STATUS_BITS) != 0u) {
            const uint32_t done = reg(USB_BUFF_STATUS_OFFSET);
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
                            out_length_[n] = static_cast<uint16_t>(mine & USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_LENGTH_0_BITS);
                            ev.out_done |= static_cast<uint16_t>(1u << n);
                        }
                    } else {
                        out_length_[n] = static_cast<uint16_t>(word & USB_DEVICE_DPRAM_EP0_OUT_BUFFER_CONTROL_LENGTH_0_BITS);
                        ev.out_done |= static_cast<uint16_t>(1u << n);
                    }
                }
            }
            hw_clear(reg(USB_BUFF_STATUS_OFFSET), done);
        }
        return ev;
    }

    // ---- readbacks ---------------------------------------------------------------------
    static bool connected() { return (reg(USB_SIE_STATUS_OFFSET) & USB_SIE_STATUS_CONNECTED_BITS) != 0u; }
    static bool suspended() { return (reg(USB_SIE_STATUS_OFFSET) & USB_SIE_STATUS_SUSPENDED_BITS) != 0u; }
    static bool vbus_detected() { return (reg(USB_SIE_STATUS_OFFSET) & USB_SIE_STATUS_VBUS_DETECTED_BITS) != 0u; }
    static bool pulled_up() { return (reg(USB_SIE_CTRL_OFFSET) & USB_SIE_CTRL_PULLUP_EN_BITS) != 0u; }
    static uint8_t address() { return static_cast<uint8_t>(reg(USB_ADDR_ENDP_OFFSET) & 0x7Fu); }
    /// The last start-of-frame number seen (a frame a millisecond while the host is awake).
    static uint16_t frame() { return static_cast<uint16_t>(reg(USB_SOF_RD_OFFSET) & 0x7FFu); }
    static uint32_t sie_status() { return reg(USB_SIE_STATUS_OFFSET); }
    /// The error flags of the SIE (CRC, bit stuff, overflow, timeout, data sequence), read and cleared.
    static uint32_t take_errors() {
        constexpr uint32_t errors = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS | USB_SIE_STATUS_RX_TIMEOUT_BITS |
                                    USB_SIE_STATUS_RX_OVERFLOW_BITS | USB_SIE_STATUS_BIT_STUFF_ERROR_BITS |
                                    USB_SIE_STATUS_CRC_ERROR_BITS;
        const uint32_t seen = reg(USB_SIE_STATUS_OFFSET) & errors;
        if (seen != 0u) {
            hw_clear(reg(USB_SIE_STATUS_OFFSET), seen);
        }
        return seen;
    }
    static uint32_t buffers_used() { return next_buffer_; }

private:
    /// The two-step write of 4.1.2.7.1: the word without AVAILABLE, a
    /// few clk_usb periods, then AVAILABLE.
    static void offer(volatile uint32_t& control, uint32_t word) {
        control = word;
        for (uint8_t spins = 0; spins < 12u; ++spins) {
            __NOP();
        }
        control = word | USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_AVAILABLE_0_BITS;
    }

    /// One half of a double-buffered endpoint's control word (the RAM
    /// takes 16-bit accesses, 4.1.2.7).
    static volatile uint16_t& buf_control_half(uint8_t number, bool in, uint8_t half) {
        return *reinterpret_cast<volatile uint16_t*>(
            static_cast<uintptr_t>(usbctrl_dpram_base + 0x80u + 8u * number + (in ? 0u : 4u) + 2u * half));
    }
    static void offer_half(volatile uint16_t& control, uint16_t word) {
        control = word;
        for (uint8_t spins = 0; spins < 12u; ++spins) {
            __NOP();
        }
        control = static_cast<uint16_t>(word | USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_AVAILABLE_0_BITS);
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
