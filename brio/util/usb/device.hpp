/*
 * usb/device.hpp
 *
 * The USB device stack, target-independent: the vocabulary of the
 * specification's chapter 9 (the setup packet, the descriptors as
 * constexpr bytes), the contract a target's endpoint controller
 * realizes (`UsbController`), the contract a class realizes
 * (`UsbClass`), and `UsbDevice`, the control-endpoint machine that
 * enumerates the device and routes everything else to its classes.
 *
 * WHY A STACK IN UTIL. Every controller brio will meet - the RP2040's
 * own, a Synopsys OTG, ST's device block, WCH's - differs in how a
 * packet is handed over (a dual-port RAM, a FIFO, a table of buffers
 * in main memory) and agrees on everything the host can see: endpoint
 * zero takes eight-byte setup packets and answers them in stages, the
 * other endpoints move packets of at most their maximum size, a bus
 * reset puts the device back at address zero. So the contract below is
 * drawn at the PACKET: a controller accepts one packet to send on an
 * endpoint and reports when it went, arms one packet's reception and
 * reports when it came with its bytes, delivers the setup packet,
 * reports the reset. Everything above the packet - the control
 * transfer's data and status stages, the descriptors, the address that
 * takes effect only after its status stage, the configuration, the
 * classes - is written once, here, and tested on the host against a
 * scripted controller (host/sim_usb.hpp) before any silicon.
 *
 * WHERE IT RUNS. The whole stack is an ISR body: `UsbDevice::isr()` is
 * what the app's controller vector calls. Enumeration is a dialogue
 * the host paces in milliseconds and every answer here is a copy of
 * constexpr bytes or a one-line state change, so nothing is deferred
 * to an active object: a class that has more to do than moving bytes
 * (a byte transport's rings, as usb/cdc.hpp) does it in the same body
 * and hands the rest to the program through its own surface. The
 * program's side of a class (write_byte, read_byte) runs in the main
 * loop under the platform's guard, the ISR side in this body: the two
 * contexts brio has, and one boundary.
 *
 * THE CONTROL TRANSFER, in the stack's words. A setup packet opens a
 * transfer and closes any pending one. A request answered with DATA
 * sends it in packets of the control endpoint's size and, when the
 * data is shorter than the host asked and ends on a packet boundary,
 * a zero-length packet after it; the host then sends an empty OUT as
 * the status stage. A request answered with RECEIVE arms the control
 * endpoint for the host's data (at most one packet here: the class
 * requests brio carries fit in one), hands it to the class, and
 * answers an empty IN as the status. A request answered with ACK
 * answers the empty IN at once. A request answered with STALL stalls
 * both directions of the control endpoint until the next setup. A new
 * ADDRESS is written to the controller only when its status stage has
 * gone, because the status itself travels at the old one.
 *
 * WHAT A CLASS IS. A type with static verbs (the file header of
 * usb/cdc.hpp is the model): the interfaces it owns, the descriptor
 * bytes the app appends to the configuration, `configure()` (the host
 * chose the configuration: claim the endpoints and arm them),
 * `reset()`, `control(setup)` for the class requests aimed at its
 * interfaces, `control_data` for the bytes a RECEIVE brought, and
 * `in_done` / `out_done` for its own endpoints' packets. `UsbDevice`
 * asks each class whether an interface or an endpoint is its.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <algorithm>
#include <array>
#include <concepts>
#include <span>

namespace brio {

// ---- the setup packet and its fields ------------------------------------------

struct UsbSetup {
    uint8_t request_type;
    uint8_t request;
    uint16_t value;
    uint16_t index;
    uint16_t length;

    static constexpr uint8_t type_standard = 0;
    static constexpr uint8_t type_class = 1;
    static constexpr uint8_t type_vendor = 2;
    static constexpr uint8_t to_device = 0;
    static constexpr uint8_t to_interface = 1;
    static constexpr uint8_t to_endpoint = 2;

    constexpr bool device_to_host() const { return (request_type & 0x80u) != 0u; }
    constexpr uint8_t type() const { return static_cast<uint8_t>((request_type >> 5) & 0x03u); }
    constexpr uint8_t recipient() const { return static_cast<uint8_t>(request_type & 0x1Fu); }
};

/// Chapter 9's standard request codes.
struct UsbRequest {
    static constexpr uint8_t get_status = 0;
    static constexpr uint8_t clear_feature = 1;
    static constexpr uint8_t set_feature = 3;
    static constexpr uint8_t set_address = 5;
    static constexpr uint8_t get_descriptor = 6;
    static constexpr uint8_t set_descriptor = 7;
    static constexpr uint8_t get_configuration = 8;
    static constexpr uint8_t set_configuration = 9;
    static constexpr uint8_t get_interface = 10;
    static constexpr uint8_t set_interface = 11;
    static constexpr uint8_t synch_frame = 12;
};

/// Descriptor types (wValue's high byte of GET_DESCRIPTOR).
struct UsbDescriptorType {
    static constexpr uint8_t device = 1;
    static constexpr uint8_t configuration = 2;
    static constexpr uint8_t string = 3;
    static constexpr uint8_t interface = 4;
    static constexpr uint8_t endpoint = 5;
    static constexpr uint8_t device_qualifier = 6;
    static constexpr uint8_t cs_interface = 0x24;   ///< a class-specific interface descriptor
};

enum class UsbEndpointType : uint8_t { control = 0, isochronous = 1, bulk = 2, interrupt = 3 };

/// Endpoint addresses as the descriptors spell them: bit 7 the direction.
constexpr uint8_t usb_ep_in(uint8_t number) { return static_cast<uint8_t>(0x80u | number); }
constexpr uint8_t usb_ep_out(uint8_t number) { return number; }
constexpr uint8_t usb_ep_number(uint8_t address) { return static_cast<uint8_t>(address & 0x0Fu); }
constexpr bool usb_ep_is_in(uint8_t address) { return (address & 0x80u) != 0u; }

/// Full speed's control and bulk packet ceiling.
inline constexpr uint16_t usb_full_speed_packet = 64;

// ---- descriptors as constexpr bytes ----------------------------------------------

/// Two or more byte arrays glued into one, at compile time.
template <size_t... Ns>
constexpr std::array<uint8_t, (Ns + ...)> usb_concat(const std::array<uint8_t, Ns>&... parts) {
    std::array<uint8_t, (Ns + ...)> out{};
    size_t at = 0;
    ((std::copy(parts.begin(), parts.end(), out.begin() + at), at += Ns), ...);
    return out;
}

/// What the device descriptor says about the device: the identity the
/// application states.
struct UsbDeviceIdentity {
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t device_version = 0x0100;   ///< BCD, 1.00
    uint8_t device_class = 0;           ///< 0: the interfaces say (the CDC wants 2, the communications class, when its two interfaces are alone)
    uint8_t device_subclass = 0;
    uint8_t device_protocol = 0;
    uint8_t manufacturer_string = 1;    ///< string descriptor indices, 0 for none
    uint8_t product_string = 2;
    uint8_t serial_string = 3;
};

/// The 18 bytes of the device descriptor (USB 2.0, full speed).
constexpr std::array<uint8_t, 18> usb_device_descriptor(const UsbDeviceIdentity& id) {
    return {18, UsbDescriptorType::device, 0x00, 0x02,   // bcdUSB 2.00
            id.device_class, id.device_subclass, id.device_protocol, static_cast<uint8_t>(usb_full_speed_packet),
            static_cast<uint8_t>(id.vendor_id), static_cast<uint8_t>(id.vendor_id >> 8),
            static_cast<uint8_t>(id.product_id), static_cast<uint8_t>(id.product_id >> 8),
            static_cast<uint8_t>(id.device_version), static_cast<uint8_t>(id.device_version >> 8),
            id.manufacturer_string, id.product_string, id.serial_string, 1};   // one configuration
}

/// The 9-byte head of the configuration descriptor; `total` is the
/// whole configuration's length, interfaces and endpoints included.
constexpr std::array<uint8_t, 9> usb_configuration_head(uint16_t total, uint8_t interfaces, bool self_powered = false,
                                                       bool remote_wakeup = false, uint16_t max_power_ma = 100) {
    const uint8_t attributes = static_cast<uint8_t>(0x80u | (self_powered ? 0x40u : 0u) | (remote_wakeup ? 0x20u : 0u));
    return {9, UsbDescriptorType::configuration, static_cast<uint8_t>(total), static_cast<uint8_t>(total >> 8),
            interfaces, 1, 0, attributes, static_cast<uint8_t>(max_power_ma / 2u)};
}

constexpr std::array<uint8_t, 9> usb_interface_descriptor(uint8_t number, uint8_t endpoints, uint8_t cls, uint8_t subclass,
                                                          uint8_t protocol, uint8_t string_index = 0, uint8_t alternate = 0) {
    return {9, UsbDescriptorType::interface, number, alternate, endpoints, cls, subclass, protocol, string_index};
}

constexpr std::array<uint8_t, 7> usb_endpoint_descriptor(uint8_t address, UsbEndpointType type, uint16_t max_packet,
                                                         uint8_t interval = 0) {
    return {7, UsbDescriptorType::endpoint, address, static_cast<uint8_t>(type), static_cast<uint8_t>(max_packet),
            static_cast<uint8_t>(max_packet >> 8), interval};
}

/// String descriptor zero: the one language, US English.
constexpr std::array<uint8_t, 4> usb_language_descriptor(uint16_t language = 0x0409) {
    return {4, UsbDescriptorType::string, static_cast<uint8_t>(language), static_cast<uint8_t>(language >> 8)};
}

/// A string descriptor from an ASCII literal: UTF-16LE, the length
/// byte first, the terminator not carried.
template <size_t N>
constexpr std::array<uint8_t, 2 + 2 * (N - 1)> usb_string_descriptor(const char (&text)[N]) {
    std::array<uint8_t, 2 + 2 * (N - 1)> out{};
    out[0] = static_cast<uint8_t>(out.size());
    out[1] = UsbDescriptorType::string;
    for (size_t i = 0; i + 1 < N; ++i) {
        out[2 + 2 * i] = static_cast<uint8_t>(text[i]);
        out[3 + 2 * i] = 0;
    }
    return out;
}

// ---- the contracts ------------------------------------------------------------------

/// What a controller reports when asked: the bus events and the
/// packets that completed since the last time, endpoint NUMBERS as
/// bit positions.
struct UsbEvents {
    bool reset = false;
    bool setup = false;
    bool suspend = false;
    bool resume = false;
    uint16_t in_done = 0;    ///< bit n: endpoint n's IN packet was taken by the host
    uint16_t out_done = 0;   ///< bit n: endpoint n's OUT packet arrived (Controller::out_data(n))
};

/**
 * The endpoint controller a target realizes. Endpoints are named by
 * NUMBER and direction here (the number, 0..15; IN or OUT), the
 * descriptor address only where the host speaks.
 *
 *  - configure_endpoint(address, type, max_packet): claim one
 *    direction of one endpoint (control endpoint zero is claimed by
 *    init()). False when the controller cannot (its memory, its
 *    count). deconfigure_endpoints(): every one but zero released.
 *  - submit_in(number, data): ONE packet, data.size() <= its maximum
 *    (zero for a zero-length packet), copied into the controller's
 *    own storage before this returns; the controller answers the next
 *    IN token with it and reports it in UsbEvents::in_done.
 *  - submit_out(number, max): arm the reception of ONE packet of at
 *    most `max`; reported in out_done, its bytes in out_data(number)
 *    until the next submit_out on that endpoint.
 *  - setup(): the last setup packet.
 *  - stall(address, on): the halt of one direction; a setup packet
 *    clears the control endpoint's by the specification, the
 *    controller sees to it.
 *  - set_address(a): the device address, effective at once.
 *  - connect(on): the pull-up that tells the host a device is there.
 *  - take_events(): what happened since the last call, acknowledged.
 */
template <typename C>
concept UsbController = requires(uint8_t address, uint8_t number, UsbEndpointType type, uint16_t max,
                                 std::span<const uint8_t> data, bool on) {
    { C::connect(on) } -> std::same_as<void>;
    { C::set_address(address) } -> std::same_as<void>;
    { C::configure_endpoint(address, type, max) } -> std::same_as<bool>;
    { C::deconfigure_endpoints() } -> std::same_as<void>;
    { C::stall(address, on) } -> std::same_as<void>;
    { C::stalled(address) } -> std::same_as<bool>;
    { C::submit_in(number, data) } -> std::same_as<bool>;
    { C::submit_out(number, max) } -> std::same_as<bool>;
    { C::out_data(number) } -> std::same_as<std::span<const uint8_t>>;
    { C::setup() } -> std::same_as<UsbSetup>;
    { C::take_events() } -> std::same_as<UsbEvents>;
};

/// A class's answer to a control request aimed at it.
struct UsbControlAnswer {
    enum class Kind : uint8_t { stall, ack, send, receive };
    Kind kind = Kind::stall;
    std::span<const uint8_t> data{};   ///< send: the bytes (the stack truncates to wLength)
    uint16_t receive_length = 0;       ///< receive: how many bytes the host will send (one packet at most)

    static constexpr UsbControlAnswer stall() { return {}; }
    static constexpr UsbControlAnswer ack() { return {Kind::ack, {}, 0}; }
    static constexpr UsbControlAnswer send(std::span<const uint8_t> d) { return {Kind::send, d, 0}; }
    static constexpr UsbControlAnswer receive(uint16_t n) { return {Kind::receive, {}, n}; }
};

/// What a class must provide (the file header).
template <typename K>
concept UsbClass = requires(const UsbSetup& s, uint8_t interface, uint8_t address, uint8_t number,
                            std::span<const uint8_t> data) {
    { K::owns_interface(interface) } -> std::same_as<bool>;
    { K::owns_endpoint(address) } -> std::same_as<bool>;
    { K::configure() } -> std::same_as<bool>;
    { K::reset() } -> std::same_as<void>;
    { K::control(s) } -> std::same_as<UsbControlAnswer>;
    { K::control_data(s, data) } -> std::same_as<void>;
    { K::in_done(number) } -> std::same_as<void>;
    { K::out_done(number, data) } -> std::same_as<void>;
};

/// The device's state as chapter 9 names it.
enum class UsbDeviceState : uint8_t { detached, defaulted, addressed, configured };

/**
 * What an application states about its device: the three descriptors
 * the standard requests answer with. `device` and `configuration` are
 * constexpr byte arrays (usb_device_descriptor, usb_configuration_head
 * glued to the classes' descriptors); `string(index)` answers the
 * string descriptors, index 0 the language list, an empty span for an
 * index it does not have.
 */
template <typename D>
concept UsbDeviceDescriptors = requires(uint8_t index) {
    { std::span<const uint8_t>(D::device) };
    { std::span<const uint8_t>(D::configuration) };
    { D::string(index) } -> std::same_as<std::span<const uint8_t>>;
};

/**
 * The device: the control endpoint's machine over a controller, the
 * classes behind it. Everything static, one device per program (a
 * chip has one controller).
 */
template <UsbController Controller, UsbDeviceDescriptors Descriptors, UsbClass... Classes>
class UsbDevice {
    static_assert(sizeof...(Classes) >= 1, "brio UsbDevice: a device with no class answers enumeration and nothing else");

public:
    UsbDevice() = delete;

    /// The controller brought up by the application (its clock, its
    /// block), then this: the state cleared, the classes reset, the
    /// pull-up raised. From here on isr() does the rest.
    static void start() {
        state_ = UsbDeviceState::defaulted;
        stage_ = Stage::idle;
        pending_address_ = 0xFF;
        (Classes::reset(), ...);
        Controller::connect(true);
    }

    static void stop() {
        Controller::connect(false);
        Controller::deconfigure_endpoints();
        (Classes::reset(), ...);
        state_ = UsbDeviceState::detached;
    }

    /// THE ISR BODY the controller's vector calls.
    [[gnu::always_inline]] static void isr() {
        const UsbEvents ev = Controller::take_events();
        if (ev.reset) {
            on_reset();
        }
        if (ev.setup) {
            on_setup(Controller::setup());
        }
        if (ev.suspend) {
            suspends_ = suspends_ + 1u;
        }
        if (ev.resume) {
            resumes_ = resumes_ + 1u;
        }
        for (uint8_t n = 0; n < 16u; ++n) {
            if ((ev.in_done & (1u << n)) != 0u) {
                if (n == 0u) {
                    ep0_in_done();
                } else {
                    (dispatch_in<Classes>(n) || ...);
                }
            }
            if ((ev.out_done & (1u << n)) != 0u) {
                const std::span<const uint8_t> data = Controller::out_data(n);
                if (n == 0u) {
                    ep0_out_done(data);
                } else {
                    (dispatch_out<Classes>(n, data) || ...);
                }
            }
        }
    }

    // ---- readbacks ---------------------------------------------------------------
    static UsbDeviceState state() { return state_; }
    static bool configured() { return state_ == UsbDeviceState::configured; }
    static uint8_t address() { return address_; }
    static uint32_t resets() { return resets_; }
    static uint32_t setups() { return setups_; }
    static uint32_t stalls() { return stalls_; }
    static uint32_t suspends() { return suspends_; }
    static uint32_t resumes() { return resumes_; }
    static uint8_t last_request() { return last_request_; }

private:
    enum class Stage : uint8_t { idle, in_data, in_zlp, status_out, out_data, status_in };

    static void on_reset() {
        resets_ = resets_ + 1u;
        Controller::deconfigure_endpoints();
        Controller::set_address(0);
        address_ = 0;
        pending_address_ = 0xFF;
        state_ = UsbDeviceState::defaulted;
        stage_ = Stage::idle;
        (Classes::reset(), ...);
    }

    static void on_setup(const UsbSetup& s) {
        setups_ = setups_ + 1u;
        last_request_ = s.request;
        stage_ = Stage::idle;
        setup_ = s;
        UsbControlAnswer answer = UsbControlAnswer::stall();
        if (s.type() == UsbSetup::type_standard) {
            answer = standard(s);
        } else if (s.recipient() == UsbSetup::to_interface) {
            const uint8_t interface = static_cast<uint8_t>(s.index);
            (route_control<Classes>(interface, s, answer) || ...);
        } else if (s.recipient() == UsbSetup::to_endpoint) {
            const uint8_t address = static_cast<uint8_t>(s.index);
            (route_control_endpoint<Classes>(address, s, answer) || ...);
        }
        apply(answer, s);
    }

    static void apply(const UsbControlAnswer& a, const UsbSetup& s) {
        switch (a.kind) {
        case UsbControlAnswer::Kind::stall:
            stalls_ = stalls_ + 1u;
            Controller::stall(usb_ep_in(0), true);
            Controller::stall(usb_ep_out(0), true);
            stage_ = Stage::idle;
            return;
        case UsbControlAnswer::Kind::ack:
            stage_ = Stage::status_in;
            (void)Controller::submit_in(0, {});
            return;
        case UsbControlAnswer::Kind::send: {
            const size_t n = a.data.size() < s.length ? a.data.size() : s.length;
            tx_ = a.data.first(n);
            tx_sent_ = 0;
            // A zero-length packet closes a data stage shorter than asked
            // that ends on a packet boundary (chapter 8's rule).
            tx_zlp_ = n < s.length && (n % usb_full_speed_packet) == 0u;
            stage_ = Stage::in_data;
            send_chunk();
            return;
        }
        case UsbControlAnswer::Kind::receive:
            if (!s.device_to_host() && a.receive_length <= sizeof rx_ && a.receive_length <= s.length) {
                rx_expected_ = a.receive_length;
                rx_got_ = 0;
                stage_ = Stage::out_data;
                (void)Controller::submit_out(0, usb_full_speed_packet);
            } else {
                apply(UsbControlAnswer::stall(), s);
            }
            return;
        }
    }

    static void send_chunk() {
        const size_t left = tx_.size() - tx_sent_;
        const size_t n = left < usb_full_speed_packet ? left : usb_full_speed_packet;
        (void)Controller::submit_in(0, tx_.subspan(tx_sent_, n));
        tx_sent_ += n;
    }

    static void ep0_in_done() {
        switch (stage_) {
        case Stage::in_data:
            if (tx_sent_ < tx_.size()) {
                send_chunk();
            } else if (tx_zlp_) {
                tx_zlp_ = false;
                stage_ = Stage::in_zlp;
                (void)Controller::submit_in(0, {});
            } else {
                stage_ = Stage::status_out;
                (void)Controller::submit_out(0, usb_full_speed_packet);
            }
            return;
        case Stage::in_zlp:
            stage_ = Stage::status_out;
            (void)Controller::submit_out(0, usb_full_speed_packet);
            return;
        case Stage::status_in:
            // The status stage of an OUT request went: an address takes
            // effect now, at the old one the status travelled at.
            if (pending_address_ != 0xFFu) {
                address_ = pending_address_;
                pending_address_ = 0xFF;
                Controller::set_address(address_);
                state_ = address_ != 0u ? UsbDeviceState::addressed : UsbDeviceState::defaulted;
            }
            stage_ = Stage::idle;
            return;
        default:
            return;
        }
    }

    static void ep0_out_done(std::span<const uint8_t> data) {
        switch (stage_) {
        case Stage::out_data: {
            const size_t room = sizeof rx_ - rx_got_;
            const size_t n = data.size() < room ? data.size() : room;
            memcpy(rx_ + rx_got_, data.data(), n);
            rx_got_ += n;
            if (rx_got_ >= rx_expected_ || data.size() < usb_full_speed_packet) {
                const uint8_t interface = static_cast<uint8_t>(setup_.index);
                const std::span<const uint8_t> got(rx_, rx_got_);
                (route_control_data<Classes>(interface, setup_, got) || ...);
                stage_ = Stage::status_in;
                (void)Controller::submit_in(0, {});
            } else {
                (void)Controller::submit_out(0, usb_full_speed_packet);
            }
            return;
        }
        case Stage::status_out:
            stage_ = Stage::idle;
            return;
        default:
            return;
        }
    }

    // ---- the standard requests -----------------------------------------------------
    static UsbControlAnswer standard(const UsbSetup& s) {
        switch (s.request) {
        case UsbRequest::get_descriptor:
            return descriptor(static_cast<uint8_t>(s.value >> 8), static_cast<uint8_t>(s.value));
        case UsbRequest::set_address:
            if (s.value > 127u || state_ == UsbDeviceState::configured) {
                return UsbControlAnswer::stall();
            }
            pending_address_ = static_cast<uint8_t>(s.value);
            return UsbControlAnswer::ack();
        case UsbRequest::set_configuration:
            if (s.value == 0u) {
                Controller::deconfigure_endpoints();
                (Classes::reset(), ...);
                state_ = address_ != 0u ? UsbDeviceState::addressed : UsbDeviceState::defaulted;
                return UsbControlAnswer::ack();
            }
            if (s.value != 1u || state_ == UsbDeviceState::defaulted) {
                return UsbControlAnswer::stall();
            }
            if (state_ == UsbDeviceState::configured) {
                Controller::deconfigure_endpoints();
                (Classes::reset(), ...);
            }
            if (!(Classes::configure() && ...)) {
                Controller::deconfigure_endpoints();
                (Classes::reset(), ...);
                return UsbControlAnswer::stall();
            }
            state_ = UsbDeviceState::configured;
            return UsbControlAnswer::ack();
        case UsbRequest::get_configuration:
            scratch_[0] = state_ == UsbDeviceState::configured ? 1u : 0u;
            return UsbControlAnswer::send(std::span<const uint8_t>(scratch_, 1));
        case UsbRequest::get_status:
            scratch_[0] = 0;
            scratch_[1] = 0;
            if (s.recipient() == UsbSetup::to_endpoint) {
                scratch_[0] = Controller::stalled(static_cast<uint8_t>(s.index)) ? 1u : 0u;
            } else if (s.recipient() == UsbSetup::to_device) {
                scratch_[0] = (Descriptors::configuration[7] & 0x40u) != 0u ? 1u : 0u;   // self powered
            }
            return UsbControlAnswer::send(std::span<const uint8_t>(scratch_, 2));
        case UsbRequest::clear_feature:
        case UsbRequest::set_feature:
            if (s.recipient() == UsbSetup::to_endpoint && s.value == 0u) {   // ENDPOINT_HALT
                Controller::stall(static_cast<uint8_t>(s.index), s.request == UsbRequest::set_feature);
                return UsbControlAnswer::ack();
            }
            if (s.recipient() == UsbSetup::to_device && s.value == 1u) {   // DEVICE_REMOTE_WAKEUP: taken, not acted on
                return UsbControlAnswer::ack();
            }
            return UsbControlAnswer::stall();
        case UsbRequest::get_interface:
            scratch_[0] = 0;
            return UsbControlAnswer::send(std::span<const uint8_t>(scratch_, 1));
        case UsbRequest::set_interface:
            return s.value == 0u ? UsbControlAnswer::ack() : UsbControlAnswer::stall();
        default:
            return UsbControlAnswer::stall();
        }
    }

    static UsbControlAnswer descriptor(uint8_t type, uint8_t index) {
        switch (type) {
        case UsbDescriptorType::device:
            return UsbControlAnswer::send(std::span<const uint8_t>(Descriptors::device));
        case UsbDescriptorType::configuration:
            return index == 0u ? UsbControlAnswer::send(std::span<const uint8_t>(Descriptors::configuration))
                               : UsbControlAnswer::stall();
        case UsbDescriptorType::string: {
            const std::span<const uint8_t> s = Descriptors::string(index);
            return s.empty() ? UsbControlAnswer::stall() : UsbControlAnswer::send(s);
        }
        default:   // a device qualifier: a full-speed-only device stalls it, as chapter 9 says
            return UsbControlAnswer::stall();
        }
    }

    // ---- routing to the classes -------------------------------------------------------
    template <typename K>
    static bool route_control(uint8_t interface, const UsbSetup& s, UsbControlAnswer& out) {
        if (!K::owns_interface(interface)) {
            return false;
        }
        out = K::control(s);
        return true;
    }
    template <typename K>
    static bool route_control_endpoint(uint8_t address, const UsbSetup& s, UsbControlAnswer& out) {
        if (!K::owns_endpoint(address)) {
            return false;
        }
        out = K::control(s);
        return true;
    }
    template <typename K>
    static bool route_control_data(uint8_t interface, const UsbSetup& s, std::span<const uint8_t> data) {
        if (!K::owns_interface(interface)) {
            return false;
        }
        K::control_data(s, data);
        return true;
    }
    template <typename K>
    static bool dispatch_in(uint8_t number) {
        if (!K::owns_endpoint(usb_ep_in(number))) {
            return false;
        }
        K::in_done(number);
        return true;
    }
    template <typename K>
    static bool dispatch_out(uint8_t number, std::span<const uint8_t> data) {
        if (!K::owns_endpoint(usb_ep_out(number))) {
            return false;
        }
        K::out_done(number, data);
        return true;
    }

    // Read by the program, written by the ISR body: volatile, so a loop
    // waiting for the configured state sees it arrive.
    static inline volatile UsbDeviceState state_ = UsbDeviceState::detached;
    static inline Stage stage_ = Stage::idle;
    static inline UsbSetup setup_{};
    static inline std::span<const uint8_t> tx_{};
    static inline size_t tx_sent_ = 0;
    static inline bool tx_zlp_ = false;
    static inline uint8_t rx_[usb_full_speed_packet]{};
    static inline size_t rx_expected_ = 0;
    static inline size_t rx_got_ = 0;
    static inline uint8_t scratch_[2]{};
    static inline volatile uint8_t address_ = 0;
    static inline uint8_t pending_address_ = 0xFF;
    static inline volatile uint8_t last_request_ = 0;
    static inline volatile uint32_t resets_ = 0;
    static inline volatile uint32_t setups_ = 0;
    static inline volatile uint32_t stalls_ = 0;
    static inline volatile uint32_t suspends_ = 0;
    static inline volatile uint32_t resumes_ = 0;
};

} // namespace brio
