// Host tests for the USB device stack (util/usb/device.hpp) and the CDC
// ACM class (util/usb/cdc.hpp) over the scripted controller
// (host/sim_usb.hpp): the test plays the host, one packet at a time.
// Run with: ctest --preset host

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include <vector>

#include "host/platform.hpp"
#include "host/sim_usb.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

using namespace brio;

namespace {

using P = HostPlatform;
using Cdc = UsbCdcAcm<SimUsb, P, 0, 1, 2, 256, 256>;

struct Descriptors {
    static constexpr auto device = usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Cdc::descriptor_bytes, Cdc::interface_count), Cdc::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto manufacturer = usb_string_descriptor("brio");
    static constexpr auto product = usb_string_descriptor("host test");
    static std::span<const uint8_t> string(uint8_t index) {
        switch (index) {
        case 0: return language;
        case 1: return manufacturer;
        case 2: return product;
        default: return {};
        }
    }
};

using Device = UsbDevice<SimUsb, Descriptors, Cdc>;

struct Long {
    static constexpr auto device = Descriptors::device;
    static constexpr auto configuration = Descriptors::configuration;
    static constexpr auto s = usb_string_descriptor("0123456789012345678901234567890");
    static std::span<const uint8_t> string(uint8_t index) { return index == 1u ? std::span<const uint8_t>(s) : std::span<const uint8_t>{}; }
};
static_assert(Long::s.size() == 64u);

constexpr UsbSetup get_descriptor(uint8_t type, uint8_t index, uint16_t length) {
    return {0x80, UsbRequest::get_descriptor, static_cast<uint16_t>((type << 8) | index), 0, length};
}

/// A control read as the host does it: the setup, IN tokens until a
/// short packet, the OUT status stage. The bytes, and how many IN
/// tokens it took.
std::vector<uint8_t> control_read(const UsbSetup& s, uint32_t* tokens = nullptr) {
    SimUsb::host_setup(s);
    Device::isr();
    std::vector<uint8_t> got;
    uint32_t n = 0;
    for (;;) {
        const auto packet = SimUsb::host_in(0);
        ++n;
        if (!packet) {
            break;   // a stall or a NAK: the caller looks at the counters
        }
        Device::isr();
        got.insert(got.end(), packet->begin(), packet->end());
        if (packet->size() < 64u) {
            break;
        }
        if (got.size() >= s.length) {
            break;
        }
    }
    (void)SimUsb::host_out(0, {});   // the status stage
    Device::isr();
    if (tokens) {
        *tokens = n;
    }
    return got;
}

/// A control write (no data, or `data`): the setup, the OUT data stage
/// if any, the IN status stage. True when the status came.
bool control_write(const UsbSetup& s, std::span<const uint8_t> data = {}) {
    SimUsb::host_setup(s);
    Device::isr();
    if (!data.empty()) {
        if (!SimUsb::host_out(0, data)) {
            return false;
        }
        Device::isr();
    }
    const auto status = SimUsb::host_in(0);
    Device::isr();
    return status.has_value() && status->empty();
}

void fresh() {
    SimUsb::clear();
    Device::stop();
    Device::start();
    SimUsb::host_reset();
    Device::isr();
}

/// The whole enumeration as Linux does it, up to configured.
void enumerate() {
    fresh();
    (void)control_read(get_descriptor(UsbDescriptorType::device, 0, 64));
    REQUIRE(control_write({0x00, UsbRequest::set_address, 7, 0, 0}));
    (void)control_read(get_descriptor(UsbDescriptorType::configuration, 0, 255));
    REQUIRE(control_write({0x00, UsbRequest::set_configuration, 1, 0, 0}));
}

} // namespace

TEST_CASE("the device descriptor is read in one packet and the address takes effect after its status") {
    fresh();
    CHECK(Device::state() == UsbDeviceState::defaulted);
    CHECK(SimUsb::pulled_up);
    uint32_t tokens = 0;
    const auto d = control_read(get_descriptor(UsbDescriptorType::device, 0, 64), &tokens);
    CHECK(d.size() == 18u);
    CHECK(d[0] == 18u);
    CHECK(d[1] == UsbDescriptorType::device);
    CHECK(d[8] == 0x09);   // vendor id low byte
    CHECK(d[9] == 0x12);
    CHECK(d[4] == 2u);     // the communications class at the device level
    CHECK(tokens == 1u);

    SimUsb::host_setup({0x00, UsbRequest::set_address, 7, 0, 0});
    Device::isr();
    CHECK(SimUsb::address == 0u);   // not yet: the status has not gone
    const auto status = SimUsb::host_in(0);
    REQUIRE(status.has_value());
    CHECK(status->empty());
    Device::isr();
    CHECK(SimUsb::address == 7u);
    CHECK(Device::address() == 7u);
    CHECK(Device::state() == UsbDeviceState::addressed);
}

TEST_CASE("the configuration descriptor spans two packets and a full-length read ends without a zero-length packet") {
    fresh();
    const uint16_t total = static_cast<uint16_t>(Descriptors::configuration.size());
    CHECK(total == 9u + Cdc::descriptor_bytes);
    CHECK(total == 67u);
    // The head alone, as the host asks first.
    uint32_t tokens = 0;
    const auto head = control_read(get_descriptor(UsbDescriptorType::configuration, 0, 9), &tokens);
    CHECK(head.size() == 9u);
    CHECK((head[2] | (head[3] << 8)) == total);
    CHECK(head[4] == 2u);   // two interfaces
    CHECK(tokens == 1u);
    // Then the whole thing.
    const auto whole = control_read(get_descriptor(UsbDescriptorType::configuration, 0, 255), &tokens);
    CHECK(whole.size() == total);
    CHECK(tokens == 2u);   // 64 + 3
    CHECK(whole[9] == 9u);
    CHECK(whole[10] == UsbDescriptorType::interface);
    CHECK(whole[14] == 0x02);   // the communications class
    CHECK(whole[total - 7] == 7u);
    CHECK(whole[total - 5] == usb_ep_in(2));   // the last endpoint: the bulk IN
}

TEST_CASE("a data stage that ends on a packet boundary short of the request gets its zero-length packet") {
    fresh();
    // A string of 31 characters is a 64-byte descriptor: asked with
    // wLength 255, it is a full packet followed by an empty one.
    using Dev = UsbDevice<SimUsb, Long, Cdc>;
    SimUsb::clear();
    Dev::start();
    SimUsb::host_setup(get_descriptor(UsbDescriptorType::string, 1, 255));
    Dev::isr();
    const auto first = SimUsb::host_in(0);
    REQUIRE(first.has_value());
    CHECK(first->size() == 64u);
    Dev::isr();
    const auto second = SimUsb::host_in(0);
    REQUIRE(second.has_value());
    CHECK(second->empty());
    Dev::isr();
    CHECK(SimUsb::out_armed(0));   // the status stage awaited
    CHECK(SimUsb::host_out(0, {}));
    Dev::isr();
    // The same string asked with wLength 64 exactly: no zero-length packet.
    SimUsb::host_setup(get_descriptor(UsbDescriptorType::string, 1, 64));
    Dev::isr();
    (void)SimUsb::host_in(0);
    Dev::isr();
    CHECK(!SimUsb::in_offered(0));
    CHECK(SimUsb::out_armed(0));
    Dev::stop();
}

TEST_CASE("strings: the language list, the two strings, and an index the device has not stalls") {
    fresh();
    const auto lang = control_read(get_descriptor(UsbDescriptorType::string, 0, 255));
    CHECK(lang.size() == 4u);
    CHECK(lang[2] == 0x09);
    CHECK(lang[3] == 0x04);
    const auto product = control_read(get_descriptor(UsbDescriptorType::string, 2, 255));
    CHECK(product.size() == 2u + 2u * 9u);
    CHECK(product[2] == 'h');
    CHECK(product[3] == 0u);
    const uint32_t stalls_before = Device::stalls();
    SimUsb::host_setup(get_descriptor(UsbDescriptorType::string, 9, 255));
    Device::isr();
    CHECK(Device::stalls() == stalls_before + 1u);
    CHECK(!SimUsb::host_in(0).has_value());
    CHECK(SimUsb::host_saw_stalls == 1u);
    // A device qualifier: a full-speed device stalls it.
    SimUsb::host_setup(get_descriptor(UsbDescriptorType::device_qualifier, 0, 10));
    Device::isr();
    CHECK(Device::stalls() == stalls_before + 2u);
    // The next setup clears the stall.
    const auto d = control_read(get_descriptor(UsbDescriptorType::device, 0, 18));
    CHECK(d.size() == 18u);
}

TEST_CASE("set configuration claims the class's three endpoints and arms the bulk OUT; configuration 2 stalls; 0 unconfigures") {
    enumerate();
    CHECK(Device::configured());
    CHECK(Cdc::configured());
    CHECK(SimUsb::configured_count == 3u);
    CHECK(SimUsb::in_[1].type == UsbEndpointType::interrupt);
    CHECK(SimUsb::out_[2].type == UsbEndpointType::bulk);
    CHECK(SimUsb::in_[2].type == UsbEndpointType::bulk);
    CHECK(SimUsb::out_armed(2));
    CHECK(!SimUsb::in_offered(2));

    const auto conf = control_read({0x80, UsbRequest::get_configuration, 0, 0, 1});
    CHECK(conf.size() == 1u);
    CHECK(conf[0] == 1u);

    const uint32_t stalls_before = Device::stalls();
    SimUsb::host_setup({0x00, UsbRequest::set_configuration, 2, 0, 0});
    Device::isr();
    CHECK(Device::stalls() == stalls_before + 1u);
    CHECK(Device::configured());   // untouched by a refused request

    REQUIRE(control_write({0x00, UsbRequest::set_configuration, 0, 0, 0}));
    CHECK(!Device::configured());
    CHECK(Device::state() == UsbDeviceState::addressed);
    CHECK(!Cdc::configured());
    CHECK(SimUsb::deconfigures >= 1u);
}

TEST_CASE("a class that cannot claim its endpoints makes SET_CONFIGURATION stall") {
    fresh();
    (void)control_read(get_descriptor(UsbDescriptorType::device, 0, 64));
    REQUIRE(control_write({0x00, UsbRequest::set_address, 3, 0, 0}));
    SimUsb::refuse_configure = true;
    const uint32_t stalls_before = Device::stalls();
    SimUsb::host_setup({0x00, UsbRequest::set_configuration, 1, 0, 0});
    Device::isr();
    CHECK(Device::stalls() == stalls_before + 1u);
    CHECK(!Device::configured());
    SimUsb::refuse_configure = false;
}

TEST_CASE("the CDC requests: the line coding both ways through a data stage, the control line state, a break") {
    enumerate();
    const uint8_t coding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};   // 115200 8N1
    REQUIRE(control_write({0x21, UsbCdcRequest::set_line_coding, 0, 0, 7}, coding));
    CHECK(Cdc::line_coding().baud == 115200u);
    CHECK(Cdc::line_coding().data_bits == 8u);
    CHECK(Cdc::coding_changes() == 1u);
    const uint8_t other[7] = {0x80, 0x25, 0x00, 0x00, 2, 2, 7};   // 9600 7E2
    REQUIRE(control_write({0x21, UsbCdcRequest::set_line_coding, 0, 0, 7}, other));
    CHECK(Cdc::line_coding().baud == 9600u);
    CHECK(Cdc::line_coding().stop_bits == 2u);
    CHECK(Cdc::line_coding().parity == 2u);
    CHECK(Cdc::line_coding().data_bits == 7u);

    const auto back = control_read({0xA1, UsbCdcRequest::get_line_coding, 0, 0, 7});
    REQUIRE(back.size() == 7u);
    CHECK(back[0] == 0x80);
    CHECK(back[1] == 0x25);
    CHECK(back[6] == 7u);

    CHECK(!Cdc::dtr());
    REQUIRE(control_write({0x21, UsbCdcRequest::set_control_line_state, 0x0003, 0, 0}));
    CHECK(Cdc::dtr());
    CHECK(Cdc::rts());
    REQUIRE(control_write({0x21, UsbCdcRequest::set_control_line_state, 0x0000, 0, 0}));
    CHECK(!Cdc::dtr());
    CHECK(Cdc::line_state_changes() == 2u);
    REQUIRE(control_write({0x21, UsbCdcRequest::send_break, 0xFFFF, 0, 0}));
    CHECK(Cdc::breaks() == 1u);

    // A class request to an interface nobody owns stalls.
    const uint32_t stalls_before = Device::stalls();
    SimUsb::host_setup({0x21, UsbCdcRequest::set_control_line_state, 0, 5, 0});
    Device::isr();
    CHECK(Device::stalls() == stalls_before + 1u);
}

TEST_CASE("bytes from the host land in the receive ring, and the endpoint is re-armed only while a packet fits") {
    enumerate();
    std::vector<uint8_t> packet(64);
    for (uint32_t i = 0; i < 64u; ++i) {
        packet[i] = static_cast<uint8_t>(i);
    }
    REQUIRE(SimUsb::host_out(2, packet));
    Device::isr();
    CHECK(Cdc::rx_bytes() == 64u);
    CHECK(SimUsb::out_armed(2));   // room for more
    // The ring holds one byte short of its size: two more packets leave
    // 63 bytes of room, less than a packet, and the endpoint stays
    // unarmed - the fourth packet is NAKed.
    static_assert(Cdc::rx_capacity() == 255u);
    REQUIRE(SimUsb::host_out(2, packet));
    Device::isr();
    REQUIRE(SimUsb::host_out(2, packet));
    Device::isr();
    CHECK(Cdc::rx_bytes() == 192u);
    CHECK(!SimUsb::out_armed(2));
    CHECK(!SimUsb::host_out(2, packet));
    CHECK(Cdc::rx_overruns() == 0u);
    // Reading frees the room: one byte makes a packet fit and the
    // endpoint is armed again.
    uint8_t b = 0;
    REQUIRE(Cdc::read_byte(b));
    CHECK(b == 0u);
    CHECK(SimUsb::out_armed(2));
    REQUIRE(SimUsb::host_out(2, packet));
    Device::isr();
    CHECK(Cdc::rx_bytes() == 256u);
    CHECK(!SimUsb::out_armed(2));
    for (uint32_t i = 1; i < 64u; ++i) {
        REQUIRE(Cdc::read_byte(b));
        CHECK(b == static_cast<uint8_t>(i));
    }
    CHECK(!SimUsb::out_armed(2));   // 64 read of 255 held: 63 bytes of room
    REQUIRE(Cdc::read_byte(b));
    CHECK(b == 0u);   // the second packet's first byte
    CHECK(SimUsb::out_armed(2));
    REQUIRE(SimUsb::host_out(2, std::span<const uint8_t>(packet.data(), 5)));
    Device::isr();
    CHECK(Cdc::rx_bytes() == 261u);
}

TEST_CASE("bytes written go out as packets: the first kicks the endpoint, completions drain the ring, a full run ends with a zero-length packet") {
    enumerate();
    CHECK(Cdc::tx_idle());
    REQUIRE(Cdc::write_byte('A'));
    CHECK(!Cdc::tx_idle());
    CHECK(SimUsb::in_offered(2));
    REQUIRE(Cdc::write_byte('B'));   // queued behind the packet in flight
    auto p = SimUsb::host_in(2);
    REQUIRE(p.has_value());
    CHECK(p->size() == 1u);
    CHECK((*p)[0] == 'A');
    Device::isr();   // the completion sends the next packet
    p = SimUsb::host_in(2);
    REQUIRE(p.has_value());
    CHECK(p->size() == 1u);
    CHECK((*p)[0] == 'B');
    Device::isr();
    CHECK(!SimUsb::in_offered(2));
    CHECK(Cdc::tx_idle());

    // Exactly 64 bytes as one write: one full packet, then an empty one.
    std::vector<uint8_t> run(100);
    for (uint32_t i = 0; i < 100u; ++i) {
        run[i] = static_cast<uint8_t>(i);
    }
    CHECK(Cdc::write(run.data(), 64) == 64u);
    p = SimUsb::host_in(2);
    REQUIRE(p.has_value());
    CHECK(p->size() == 64u);
    Device::isr();
    p = SimUsb::host_in(2);
    REQUIRE(p.has_value());
    CHECK(p->empty());
    Device::isr();
    CHECK(!SimUsb::in_offered(2));
    CHECK(Cdc::tx_bytes() == 66u);

    // 100 bytes: 64 + 36, no empty packet.
    CHECK(Cdc::write(run.data(), 100) == 100u);
    p = SimUsb::host_in(2);
    REQUIRE(p.has_value());
    CHECK(p->size() == 64u);
    Device::isr();
    p = SimUsb::host_in(2);
    REQUIRE(p.has_value());
    CHECK(p->size() == 36u);
    Device::isr();
    CHECK(!SimUsb::in_offered(2));
    CHECK(Cdc::tx_idle());

    // A full ring refuses the byte that does not fit, like a UART's: the
    // first byte left at the kick as a packet of its own, the ring
    // holds the rest.
    uint32_t accepted = 0;
    while (Cdc::write_byte(0x55)) {
        ++accepted;
    }
    CHECK(accepted == 1u + Cdc::tx_capacity());
    CHECK(SimUsb::double_submits == 0u);
}

TEST_CASE("unconfigured, the transport refuses bytes; a bus reset unconfigures and the stack recovers") {
    fresh();
    CHECK(!Cdc::write_byte('x'));
    enumerate();
    REQUIRE(Cdc::write_byte('x'));
    SimUsb::host_reset();
    Device::isr();
    CHECK(Device::resets() >= 1u);
    CHECK(Device::state() == UsbDeviceState::defaulted);
    CHECK(Device::address() == 0u);
    CHECK(SimUsb::address == 0u);
    CHECK(!Cdc::configured());
    CHECK(Cdc::tx_idle());
    (void)control_read(get_descriptor(UsbDescriptorType::device, 0, 64));
    REQUIRE(control_write({0x00, UsbRequest::set_address, 9, 0, 0}));
    REQUIRE(control_write({0x00, UsbRequest::set_configuration, 1, 0, 0}));
    CHECK(Device::configured());
    CHECK(Device::address() == 9u);
}

TEST_CASE("the standard odds and ends: status, endpoint halt through the feature requests, interface, unknown request") {
    enumerate();
    auto st = control_read({0x80, UsbRequest::get_status, 0, 0, 2});
    REQUIRE(st.size() == 2u);
    CHECK(st[0] == 0u);   // bus powered, no remote wakeup
    st = control_read({0x82, UsbRequest::get_status, 0, usb_ep_in(2), 2});
    REQUIRE(st.size() == 2u);
    CHECK(st[0] == 0u);
    REQUIRE(control_write({0x02, UsbRequest::set_feature, 0, usb_ep_in(2), 0}));   // ENDPOINT_HALT
    CHECK(SimUsb::stalled(usb_ep_in(2)));
    st = control_read({0x82, UsbRequest::get_status, 0, usb_ep_in(2), 2});
    CHECK(st[0] == 1u);
    REQUIRE(control_write({0x02, UsbRequest::clear_feature, 0, usb_ep_in(2), 0}));
    CHECK(!SimUsb::stalled(usb_ep_in(2)));
    const auto alt = control_read({0x81, UsbRequest::get_interface, 0, 1, 1});
    REQUIRE(alt.size() == 1u);
    CHECK(alt[0] == 0u);
    REQUIRE(control_write({0x01, UsbRequest::set_interface, 0, 1, 0}));
    const uint32_t stalls_before = Device::stalls();
    SimUsb::host_setup({0x01, UsbRequest::set_interface, 1, 1, 0});
    Device::isr();
    CHECK(Device::stalls() == stalls_before + 1u);
    SimUsb::host_setup({0x00, 0x42, 0, 0, 0});   // no such standard request
    Device::isr();
    CHECK(Device::stalls() == stalls_before + 2u);
    SimUsb::host_suspend();
    SimUsb::host_resume();
    Device::isr();
    CHECK(Device::suspends() == 1u);
    CHECK(Device::resumes() == 1u);
}
