/*
 * usb/cdc.hpp
 *
 * CDC ACM, the communications class as a serial port: two interfaces
 * (the communications interface with its notification endpoint, the
 * data interface with a bulk pair), the three requests every host
 * driver sends (the line coding both ways, the control line state),
 * and - the point of it - a BYTE TRANSPORT over the bulk pair with the
 * same surface as a UART's (`write_byte`, `read_byte`, `tx_idle`), so
 * a console or a SerialPort runs over the chip's own connector with
 * nothing above it changed.
 *
 * THE TWO SIDES. The program's side pushes into a transmit ring and
 * pops from a receive ring under the platform's guard; the stack's
 * side, in the controller's ISR body, moves packets: a completed IN
 * takes the next packet out of the transmit ring and hands it to the
 * controller (a zero-length packet after a full one when the ring ran
 * dry, so the host's reader is not left waiting for a short packet),
 * a completed OUT lands in the receive ring and the endpoint is re-armed
 * ONLY WHEN THE RING HAS A PACKET'S ROOM - the host sees NAKs meanwhile,
 * which is USB's own flow control and costs no byte. `read_byte`
 * re-arms the endpoint when it frees the room.
 *
 * THE LINE CODING IS RECEIVED AND REPORTED, NOT OBEYED: a virtual
 * port has no baud rate. The program reads what the host set
 * (`line_coding()`, `dtr()`, `rts()`) and does with it what it likes -
 * a bridge to a real UART would apply it, a console ignores it. The
 * notification endpoint is declared because the class demands one and
 * never written: no serial state is signalled.
 *
 * ENDPOINTS AND INTERFACES are template parameters so two ports can
 * share a device: `interface` is the communications interface's
 * number (the data interface is the next), `ep_notify` the
 * notification endpoint's number (IN), `ep_data` the bulk pair's
 * number (IN and OUT). The descriptor bytes the application glues
 * into its configuration are `descriptors`.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <array>
#include <span>

#include "kernel/platform.hpp"
#include "util/ring.hpp"
#include "util/usb/device.hpp"

namespace brio {

/// What SET_LINE_CODING carries: the host's idea of the serial line.
struct UsbLineCoding {
    uint32_t baud = 115200;
    uint8_t stop_bits = 0;   ///< 0 = 1, 1 = 1.5, 2 = 2
    uint8_t parity = 0;      ///< 0 none, 1 odd, 2 even, 3 mark, 4 space
    uint8_t data_bits = 8;
};

/// The class-specific requests of the ACM subclass.
struct UsbCdcRequest {
    static constexpr uint8_t set_line_coding = 0x20;
    static constexpr uint8_t get_line_coding = 0x21;
    static constexpr uint8_t set_control_line_state = 0x22;
    static constexpr uint8_t send_break = 0x23;
};

template <UsbController Controller, Platform P, uint8_t interface = 0, uint8_t ep_notify = 1, uint8_t ep_data = 2,
          uint32_t rx_size = 256, uint32_t tx_size = 256>
class UsbCdcAcm {
    static_assert(ep_notify != 0u && ep_data != 0u && ep_notify != ep_data && ep_notify < 16u && ep_data < 16u,
                  "brio UsbCdcAcm: two endpoint numbers of 1..15, the notification's and the bulk pair's, distinct");
    static_assert(rx_size >= 2u * usb_full_speed_packet && tx_size >= usb_full_speed_packet,
                  "brio UsbCdcAcm: the receive ring holds two packets at least, the transmit ring one");

public:
    constexpr UsbCdcAcm() = default;   // a tag for print(), like a Uart

    static constexpr uint8_t comm_interface = interface;
    static constexpr uint8_t data_interface = interface + 1;
    static constexpr uint8_t interface_count = 2;
    static constexpr uint16_t packet = usb_full_speed_packet;

    /// The bytes the application glues after usb_configuration_head():
    /// the communications interface (class 2, subclass 2 ACM, protocol
    /// 1 AT-commands as every host driver expects) with its four
    /// functional descriptors and the notification endpoint, then the
    /// data interface (class 10) with the bulk pair.
    static constexpr auto descriptors = usb_concat(
        usb_interface_descriptor(comm_interface, 1, 0x02, 0x02, 0x01),
        std::array<uint8_t, 5>{5, UsbDescriptorType::cs_interface, 0x00, 0x10, 0x01},          // header, CDC 1.10
        std::array<uint8_t, 5>{5, UsbDescriptorType::cs_interface, 0x01, 0x00, data_interface},   // call management: none, over the data interface
        std::array<uint8_t, 4>{4, UsbDescriptorType::cs_interface, 0x02, 0x02},                // ACM: line coding and state, break
        std::array<uint8_t, 5>{5, UsbDescriptorType::cs_interface, 0x06, comm_interface, data_interface},   // union
        usb_endpoint_descriptor(usb_ep_in(ep_notify), UsbEndpointType::interrupt, 8, 16),
        usb_interface_descriptor(data_interface, 2, 0x0A, 0x00, 0x00),
        usb_endpoint_descriptor(usb_ep_out(ep_data), UsbEndpointType::bulk, packet),
        usb_endpoint_descriptor(usb_ep_in(ep_data), UsbEndpointType::bulk, packet));
    static constexpr uint16_t descriptor_bytes = static_cast<uint16_t>(descriptors.size());

    // ---- the UsbClass contract --------------------------------------------------------
    static bool owns_interface(uint8_t i) { return i == comm_interface || i == data_interface; }
    static bool owns_endpoint(uint8_t address) {
        return address == usb_ep_in(ep_notify) || address == usb_ep_in(ep_data) || address == usb_ep_out(ep_data);
    }

    static bool configure() {
        if (!Controller::configure_endpoint(usb_ep_in(ep_notify), UsbEndpointType::interrupt, 8) ||
            !Controller::configure_endpoint(usb_ep_out(ep_data), UsbEndpointType::bulk, packet) ||
            !Controller::configure_endpoint(usb_ep_in(ep_data), UsbEndpointType::bulk, packet)) {
            return false;
        }
        configured_ = true;
        in_armed_ = false;
        out_armed_ = false;
        outs_armed_ = 0;
        last_full_ = false;
        arm_out();
        return true;
    }

    static void reset() {
        configured_ = false;
        in_armed_ = false;
        out_armed_ = false;
        outs_armed_ = 0;
        last_full_ = false;
        dtr_ = false;
        rts_ = false;
        m_rx.clear();
        m_tx.clear();
    }

    static UsbControlAnswer control(const UsbSetup& s) {
        if (s.type() != UsbSetup::type_class) {
            return UsbControlAnswer::stall();
        }
        switch (s.request) {
        case UsbCdcRequest::set_line_coding:
            return UsbControlAnswer::receive(7);
        case UsbCdcRequest::get_line_coding:
            coding_bytes_[0] = static_cast<uint8_t>(coding_.baud);
            coding_bytes_[1] = static_cast<uint8_t>(coding_.baud >> 8);
            coding_bytes_[2] = static_cast<uint8_t>(coding_.baud >> 16);
            coding_bytes_[3] = static_cast<uint8_t>(coding_.baud >> 24);
            coding_bytes_[4] = coding_.stop_bits;
            coding_bytes_[5] = coding_.parity;
            coding_bytes_[6] = coding_.data_bits;
            return UsbControlAnswer::send(std::span<const uint8_t>(coding_bytes_, 7));
        case UsbCdcRequest::set_control_line_state:
            dtr_ = (s.value & 0x01u) != 0u;
            rts_ = (s.value & 0x02u) != 0u;
            line_state_changes_ = line_state_changes_ + 1u;
            return UsbControlAnswer::ack();
        case UsbCdcRequest::send_break:
            breaks_ = breaks_ + 1u;
            return UsbControlAnswer::ack();
        default:
            return UsbControlAnswer::stall();
        }
    }

    static void control_data(const UsbSetup& s, std::span<const uint8_t> data) {
        if (s.request == UsbCdcRequest::set_line_coding && data.size() >= 7u) {
            coding_.baud = static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                           (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
            coding_.stop_bits = data[4];
            coding_.parity = data[5];
            coding_.data_bits = data[6];
            coding_changes_ = coding_changes_ + 1u;
        }
    }

    /// An IN packet went: the next one from the ring, or the zero-length
    /// packet that closes a run of full ones, or nothing.
    static void in_done(uint8_t number) {
        if (number != ep_data) {
            return;   // the notification endpoint: never written
        }
        in_armed_ = false;
        if (!m_tx.empty()) {
            send_packet();
        } else if (last_full_) {
            last_full_ = false;
            in_armed_ = Controller::submit_in(ep_data, {});
        }
    }

    /// An OUT packet came: into the ring, and the endpoint re-armed if
    /// a whole packet fits behind it.
    static void out_done(uint8_t number, std::span<const uint8_t> data) {
        if (number != ep_data) {
            return;
        }
        if (outs_armed_ != 0u) {
            outs_armed_ = outs_armed_ - 1u;
        }
        out_armed_ = outs_armed_ != 0u;
        if (m_rx.empty() && !data.empty()) {
            rx_edge_ = true;   // the ring's empty -> non-empty edge: what a SerialPort wants posted
        }
        for (const uint8_t b : data) {
            if (!m_rx.push(b)) {
                m_rx_overruns = m_rx_overruns + 1;   // cannot happen while arm_out() keeps a packet's room
            }
        }
        m_rx_bytes += static_cast<uint32_t>(data.size());
        arm_out();
    }

    // ---- the byte transport (the program's side) --------------------------------------
    /// TRY semantics like a UART's: false when the ring is full or the
    /// host has not configured the port; a queued byte goes out on the
    /// next IN token.
    static bool write_byte(uint8_t b) {
        if (!configured_ || !m_tx.push(b)) {
            return false;
        }
        kick();
        return true;
    }

    static bool read_byte(uint8_t& b) {
        const std::optional<uint8_t> v = m_rx.pop();
        if (!v) {
            return false;
        }
        b = *v;
        if (outs_armed_ < out_slots()) {
            typename P::CriticalSection cs;
            arm_out();
        }
        return true;
    }

    /// A run of bytes queued first and kicked once, so the first packet
    /// carries as many of them as fit; write_byte alone would send the
    /// first byte on its own and batch the rest behind it.
    static uint32_t write(const uint8_t* buffer, uint32_t len) {
        if (!configured_) {
            return 0;
        }
        uint32_t written = 0;
        while (written < len && m_tx.push(buffer[written])) {
            ++written;
        }
        if (written != 0u) {
            kick();
        }
        return written;
    }

    static constexpr uint32_t rx_capacity() { return Ring<uint8_t, rx_size, P>::capacity(); }
    static constexpr uint32_t tx_capacity() { return Ring<uint8_t, tx_size, P>::capacity(); }

    /// Nothing queued and nothing in flight.
    static bool tx_idle() { return m_tx.empty() && !in_armed_; }

    /// ISR glue: did a packet just fill an empty receive ring? Consumed
    /// by the read; the app posts a SerialPort's RxActivity on it, the
    /// way a UART's isr() returns the edge.
    static bool take_rx_edge() {
        const bool edge = rx_edge_;
        rx_edge_ = false;
        return edge;
    }

    /// How many OUT packets the controller takes at once (its double
    /// buffering, if it has any); one otherwise.
    static constexpr uint8_t out_slots() {
        if constexpr (requires { Controller::out_slots; }) {
            return Controller::out_slots;
        } else {
            return 1;
        }
    }

    // ---- readbacks ---------------------------------------------------------------------
    static bool configured() { return configured_; }
    static bool dtr() { return dtr_; }
    static bool rts() { return rts_; }
    static UsbLineCoding line_coding() { return coding_; }
    static uint32_t coding_changes() { return coding_changes_; }
    static uint32_t line_state_changes() { return line_state_changes_; }
    static uint32_t breaks() { return breaks_; }
    static uint32_t rx_bytes() { return m_rx_bytes; }
    static uint32_t tx_bytes() { return m_tx_bytes; }
    static uint8_t rx_overruns() { return m_rx_overruns; }
    static bool in_armed() { return in_armed_; }
    static bool out_armed() { return out_armed_; }

private:
    /// Under the guard: the first packet of a run, when nothing is in
    /// flight. The ISR side takes over from the completion on.
    static void kick() {
        typename P::CriticalSection cs;
        if (!in_armed_ && configured_) {
            send_packet();
        }
    }

    /// ISR side or under the guard: one packet out of the ring.
    static void send_packet() {
        uint16_t n = 0;
        while (n < packet) {
            const std::optional<uint8_t> v = m_tx.pop();
            if (!v) {
                break;
            }
            packet_[n++] = *v;
        }
        if (n == 0u) {
            return;
        }
        last_full_ = n == packet;
        m_tx_bytes += n;
        in_armed_ = Controller::submit_in(ep_data, std::span<const uint8_t>(packet_, n));
    }

    /// ISR side or under the guard: the OUT endpoint armed, as many
    /// packets as the controller takes at once, while each fits in the
    /// ring behind the ones already armed.
    static void arm_out() {
        if (!configured_) {
            return;
        }
        while (outs_armed_ < out_slots() &&
               static_cast<uint32_t>(m_rx.capacity() - m_rx.count()) >= static_cast<uint32_t>(packet) * (outs_armed_ + 1u)) {
            if (!Controller::submit_out(ep_data, packet)) {
                break;
            }
            outs_armed_ = outs_armed_ + 1u;
        }
        out_armed_ = outs_armed_ != 0u;
    }

    static inline Ring<uint8_t, rx_size, P> m_rx{};
    static inline Ring<uint8_t, tx_size, P> m_tx{};
    static inline uint8_t packet_[usb_full_speed_packet]{};
    static inline uint8_t coding_bytes_[7]{};
    static inline UsbLineCoding coding_{};
    static inline volatile bool configured_ = false;
    static inline volatile bool in_armed_ = false;
    static inline volatile bool out_armed_ = false;
    static inline volatile uint8_t outs_armed_ = 0;
    static inline bool last_full_ = false;
    static inline bool rx_edge_ = false;
    static inline volatile bool dtr_ = false;
    static inline volatile bool rts_ = false;
    static inline volatile uint32_t coding_changes_ = 0;
    static inline volatile uint32_t line_state_changes_ = 0;
    static inline volatile uint32_t breaks_ = 0;
    static inline volatile uint32_t m_rx_bytes = 0;
    static inline volatile uint32_t m_tx_bytes = 0;
    static inline volatile uint8_t m_rx_overruns = 0;
};

} // namespace brio
