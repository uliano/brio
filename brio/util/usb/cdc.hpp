/*
 * usb/cdc.hpp
 *
 * CDC ACM (the communications device class's abstract control model:
 * the class a host's serial-port driver binds to), the communications
 * class as a serial port: two interfaces (the communications interface
 * with its notification endpoint, the data interface with a bulk pair),
 * the three requests every host driver sends (the line coding both ways,
 * the control line state), and - the point of it - a byte transport
 * over the bulk pair with the same surface as a UART's (`write_byte`,
 * `write_bulk`, `read_byte`, `read_span`/`consume`, `tx_idle`), so a
 * console or a SerialPort runs over the chip's own connector with
 * nothing above it changed.
 *
 * The smallest use, a serial port over the RP2040's own controller:
 * the class over the controller, a SerialPort over the class, and the
 * receive edge posted from the controller's vector after the stack has
 * run (usb/device.hpp shows the descriptors and the device around it;
 * a tag object is an empty value that stands for its type in a call,
 * so print(serial, ...) knows where to write):
 *
 *     using Serial = brio::UsbCdcAcm<brio::Usb, P>;   // interface 0, endpoints 1 and 2
 *     constexpr Serial serial;                         // the tag object print() takes
 *     using SerialLines = brio::SerialPort<Serial, P, Console, 80>;
 *
 *     extern "C" void isr_usbctrl() {
 *         Device::isr();
 *         if (Serial::take_rx_edge()) { brio::post<SerialLines>(brio::RxActivity{}); }
 *     }
 *
 * Who calls what, and from where:
 *  - the contract verbs (owns_*, configure, reset, control,
 *    control_data, in_done, out_done) are UsbDevice's, called inside its
 *    isr(); reset() is also called from main context, by start() before
 *    the pull-up rises and by stop() after it drops. reset() drops the
 *    stack's side of the port - the configured flag, the packets armed,
 *    the control lines - and TOUCHES NEITHER RING: the rings are the
 *    program's, each index written by one party only (util/ring.hpp's
 *    rule), and a clear from the handler would write the program's index
 *    under a write_bulk() or a consume() in progress. So a bus reset, a
 *    deconfiguration, stop() and start() lose only the one packet the
 *    controller held (at most 64 bytes popped for it): what was queued
 *    behind it leaves when the host configures the port again (configure()
 *    hands the first packet over itself), and what was received stays
 *    readable; a program that wants them emptied calls discard(), main
 *    context, with the device stopped. take_rx_edge() belongs in the
 *    controller's vector, after Device::isr();
 *  - the byte transport is the program's, from main context: one
 *    producer of the transmit ring (write_byte, write_bulk, and print
 *    over them) and one consumer of the receive ring (read_byte,
 *    read_span/consume, or a SerialPort draining it), as util/ring.hpp's
 *    rule asks. None of these verbs waits (a print over them does,
 *    below). Every write_byte or write_bulk that queued bytes takes the
 *    platform's critical section (interrupts masked) and, when no packet
 *    is in flight, gathers the first one from the ring, up to 64 bytes,
 *    and hands it to the controller inside it; read_byte and consume
 *    take it whenever fewer receptions stand armed than the controller
 *    takes at once (out_slots()), and inside it arm receptions up to
 *    that number while the ring has a whole packet's room behind each -
 *    possibly none, the section taken all the same;
 *  - write_byte and write_bulk refuse everything while the host has not
 *    configured the port, so a print over it (util/print.hpp waits until
 *    every byte is taken) waits until the host configures it, in main
 *    context with the other active objects held, and for ever if no
 *    host does (the USB handler keeps running meanwhile);
 *  - the readbacks are snapshots of what the handler wrote, from main
 *    context.
 *
 * The two sides. The program's side pushes into a transmit ring and
 * pops from a receive ring under the platform's guard; the stack's
 * side, in the controller's ISR body, moves packets: a completed IN
 * takes the next packet out of the transmit ring and hands it to the
 * controller (a zero-length packet after a full one when the ring ran
 * dry, so the host's reader is not left waiting for a short packet),
 * a completed OUT lands in the receive ring and the endpoint is re-armed
 * only when the ring has a packet's room - the host sees NAKs (the
 * device's "not now", after which the host retries) meanwhile, which is
 * USB's own flow control and costs no byte. `read_byte` and `consume`
 * re-arm the endpoint when they free the room.
 *
 * The line coding is received and reported, not obeyed: a virtual
 * port has no baud rate. The program reads what the host set
 * (`line_coding()`, `dtr()`, `rts()`) and does with it what it likes -
 * a bridge to a real UART would apply it, a console ignores it. The
 * notification endpoint is declared because the class demands one and
 * never written: no serial state is signalled.
 *
 * Endpoints and interfaces are template parameters so two ports can
 * share a device: `interface` is the communications interface's
 * number (the data interface is the next), `ep_notify` the
 * notification endpoint's number (IN), `ep_data` the bulk pair's
 * number (IN and OUT). The descriptor bytes the application glues
 * into its configuration are `descriptors`. The rings are rx_size and
 * tx_size bytes of storage (256 each by default) and hold one byte less
 * each (util/ring.hpp: capacity is size - 1). The assertion asks for at
 * least 128 and 64 bytes of storage, so the smallest receive ring holds
 * 127 bytes, one full packet and 63 of a second (a reception is armed
 * only while a whole packet's room is free), and the smallest transmit
 * ring 63, one packet less a byte.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <array>
#include <span>

#include "kernel/platform.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"
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
                  "brio UsbCdcAcm: rx_size is two packets of storage at least and tx_size one, each ring holding a byte less");

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
        // Bytes a reset left queued (behind the packet it took, or pushed
        // in the moment between write_byte's check of configured_ and its
        // push) leave from here: the ISR side owns the transmit ring's
        // consumer index and nothing is in flight.
        if (!m_tx.empty()) {
            send_packet();
        }
        return true;
    }

    /// The stack's side dropped, the rings untouched (the file header):
    /// from isr() at a bus reset or a deconfiguration, from start() and
    /// stop() in main context.
    static void reset() {
        configured_ = false;
        in_armed_ = false;
        out_armed_ = false;
        outs_armed_ = 0;
        last_full_ = false;
        dtr_ = false;
        rts_ = false;
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
    /// Try semantics like a UART's (the call answers at once, never
    /// waits): false when the ring is full or the host has not configured
    /// the port. A queued byte is handed to the controller at once when no
    /// packet is in flight; otherwise each completed IN takes the oldest
    /// 64 bytes of the ring as the next packet, so the byte goes with the
    /// packet after the one in flight when fewer than 64 bytes are queued
    /// ahead of it, and with a later one otherwise. It travels when the
    /// host next asks the endpoint for data (an IN token, the host's
    /// poll). Main context, the transmit ring's one producer.
    static bool write_byte(uint8_t b) {
        if (!configured_ || !m_tx.push(b)) {
            return false;
        }
        kick();
        return true;
    }

    /// The oldest received byte into `b`; false when none is queued. Main
    /// context, the receive ring's one consumer. Takes the guard whenever
    /// fewer receptions stand armed than out_slots(), and re-arms the OUT
    /// endpoint inside it when the room freed takes a packet.
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

    /// A run (util/stream.hpp's BulkSink): as many of the bytes as the
    /// ring has room for, copied into its free run, then kicked once, so
    /// the first packet carries as many of them as fit - write_byte alone
    /// would send the first byte on its own and batch the rest behind it.
    /// Never blocks; returns how many were queued, none while the host has
    /// not configured the port. The bytes are copied: `src` is free when
    /// this returns.
    static uint32_t write_bulk(std::span<const uint8_t> src) {
        if (!configured_) {
            return 0;
        }
        uint32_t queued = 0;
        while (queued < src.size()) {
            const auto room = m_tx.write_span();
            if (room.empty()) {
                break;
            }
            const uint32_t want = static_cast<uint32_t>(src.size()) - queued;
            const uint32_t take = want < room.size() ? want : static_cast<uint32_t>(room.size());
            // Two pointers and no index, and the test at the bottom: a
            // load, a store, two steps and one branch a byte, where an index
            // re-adds both bases every byte. `take` is at least one here -
            // the room is not empty and the run is not done.
            const uint8_t* from = src.data() + queued;
            uint8_t* to = room.data();
            uint8_t* const end = to + take;
            do {
                *to++ = *from++;
            } while (to != end);
            m_tx.publish(static_cast<typename decltype(m_tx)::index_t>(take));
            queued += take;
        }
        if (queued != 0u) {
            kick();
        }
        return queued;
    }

    /// The received bytes in place: the contiguous run ready to be read,
    /// never wrapping - the receive ring's consumer half under the ring's
    /// own names. With consume() it is util/stream.hpp's SpanSource,
    /// which SerialPort drains a run at a time. The span stays valid until
    /// the next consume() or read_byte(): nothing on the stack's side
    /// moves the receive ring's consumer index, a bus reset included.
    static std::span<const uint8_t> read_span() { return m_rx.read_span(); }

    /// Release the first `count` bytes of read_span(), oldest first, clamped
    /// to what is queued - and, as read_byte() does, take the guard while
    /// fewer receptions stand armed than out_slots() and re-arm the OUT
    /// endpoint when the room freed takes a packet.
    static void consume(uint32_t count) {
        constexpr uint32_t most = decltype(m_rx)::capacity();
        m_rx.consume(static_cast<typename decltype(m_rx)::index_t>(count < most ? count : most));
        if (outs_armed_ < out_slots()) {
            typename P::CriticalSection cs;
            arm_out();
        }
    }

    /// Both rings emptied, the program's choice and the one write of both
    /// indices of a ring (util/ring.hpp's clear() is not concurrent): main
    /// context, with the controller's handler kept out - the device
    /// stopped (UsbDevice::stop(), the pull-up down, so no configuration
    /// can arrive and kick the queue) or its line masked. Nothing else
    /// ever empties them: not a bus reset, not stop() or start().
    static void discard() {
        m_rx.clear();
        m_tx.clear();
    }

    static constexpr uint32_t rx_capacity() { return Ring<uint8_t, rx_size, P>::capacity(); }
    static constexpr uint32_t tx_capacity() { return Ring<uint8_t, tx_size, P>::capacity(); }

    /// Nothing queued and nothing in flight: the last packet handed to the
    /// controller has been taken by the host.
    static bool tx_idle() { return m_tx.empty() && !in_armed_; }

    /// ISR glue: did a packet just fill an empty receive ring? Consumed
    /// by the read; the app posts a SerialPort's RxActivity on it, the
    /// way a UART's isr() returns the edge. Call it in the controller's
    /// vector, after Device::isr().
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
    // configured() is true from the host's SET_CONFIGURATION to the next
    // bus reset, deconfiguration or UsbDevice::stop(), and across a
    // SET_CONFIGURATION repeated while configured (the stack resets the
    // class and configures it again). Each of those loses the one packet
    // the controller held and no byte of either ring (the file header):
    // tx_idle() stays false over the queued bytes until a host takes
    // them; dtr() and rts() are the control line state the host last set,
    // false again after each.
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
