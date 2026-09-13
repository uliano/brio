// USB family smoke TU: the controller's verbs, the stack and the CDC
// class over it, clk_usb's generator.
#include "rp2040/clock.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/usb.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
using P = Rp2040Platform<>;
using Cdc = UsbCdcAcm<Usb, P>;

struct Descriptors {
    static constexpr auto device = usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Cdc::descriptor_bytes, Cdc::interface_count), Cdc::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto product = usb_string_descriptor("family");
    static std::span<const uint8_t> string(uint8_t index) {
        return index == 0u ? std::span<const uint8_t>(language) : index == 2u ? std::span<const uint8_t>(product) : std::span<const uint8_t>{};
    }
};
using Device = UsbDevice<Usb, Descriptors, Cdc>;

static_assert(UsbController<Usb> && UsbClass<Cdc> && UsbDeviceDescriptors<Descriptors>);
static_assert(Descriptors::configuration.size() == 67u);
static_assert(usb_string_descriptor("ab").size() == 6u);
static_assert(usb_ep_in(2) == 0x82u && usb_ep_number(0x82) == 2u && usb_ep_is_in(0x82));

void usb_verbs() {
    constexpr SysClock clock;
    (void)Usb::init(clock);
    Device::start();
    Device::isr();
    (void)Device::state();
    (void)Device::configured();
    (void)Device::address();
    (void)Device::resets();
    (void)Device::setups();
    (void)Device::stalls();
    (void)Device::suspends();
    (void)Device::resumes();
    (void)Device::last_request();
    uint8_t b = 0;
    (void)Cdc::write_byte('a');
    (void)Cdc::read_byte(b);
    (void)Cdc::write(&b, 1);
    (void)Cdc::tx_idle();
    (void)Cdc::dtr();
    (void)Cdc::rts();
    (void)Cdc::line_coding();
    (void)Cdc::coding_changes();
    (void)Cdc::rx_bytes();
    (void)Cdc::tx_bytes();
    (void)Cdc::rx_overruns();
    (void)Usb::connected();
    (void)Usb::suspended();
    (void)Usb::vbus_detected();
    (void)Usb::pulled_up();
    (void)Usb::address();
    (void)Usb::frame();
    (void)Usb::sie_status();
    (void)Usb::take_errors();
    (void)Usb::buffers_used();
    Device::stop();
    Usb::release();
    Clocks::usb_select(UsbAux::pll_usb);
    (void)Clocks::usb_enabled();
    (void)Clocks::usb_source();
    Clocks::usb_stop();
}
