// mcu: ch32v203f6 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// USBD family smoke TU: the device controller as util/usb's
// UsbController, the whole stack composed over it - the control-endpoint
// machine and a CDC ACM port - and every verb of the driver.
//
// Eight CH32V203 of the nine: the CH32V203F8's package bonds neither
// PA11 nor PA12, so the datasheet gives it no USB device controller at
// all, and the CH32V303 has none either (its one full-speed controller
// is RM ch. 23's USBFS) - a neg TU proves the refusal on both.
#include "ch32v203/clock.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/usb.hpp"
#include "util/stream.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

#include <span>

using namespace brio;

static_assert(device::has_usbd);

using P = Ch32v203Platform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;
using Usb = Usbd<>;
using Small = Usbd<128>;
using Serial = UsbCdcAcm<Usb, P>;

// The claim the driver makes, once more where the fixture can see it.
static_assert(UsbController<Usb>);
static_assert(UsbClass<Serial>);
static_assert(ByteSink<Serial> && ByteSource<Serial>);

// The shared memory is 512 bytes and the CAN filter table takes the top
// 128 when CAN is used, which is why the default budget is 384.
static_assert(Usb::max_packet == 64);
static_assert(Usb::btable_offset == 0 && Usb::buffer_floor == 64);

// The bus has a measured floor of its own, and this tree clears it; a
// neg TU proves the refusal below it.
static_assert(SysClock::hz >= usbd_min_hclk_hz);

struct Descriptors {
    static constexpr auto device =
        usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Serial::descriptor_bytes, Serial::interface_count),
                   Serial::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto product = usb_string_descriptor("brio family check");
    static std::span<const uint8_t> string(uint8_t index) {
        switch (index) {
        case 0: return language;
        case 1: return product;
        default: return {};
        }
    }
};

using Device = UsbDevice<Usb, Descriptors, Serial>;

void usb_verbs() {
    constexpr SysClock clock;
    (void)Usb::init(clock);
    (void)Small::init(clock);

    Usb::connect(true);
    Usb::set_address(7);
    (void)Usb::configure_endpoint(0x81, UsbEndpointType::interrupt, 8);
    (void)Usb::configure_endpoint(0x02, UsbEndpointType::bulk, 64);
    Usb::stall(0x81, true);
    (void)Usb::stalled(0x81);
    Usb::stall(0x81, false);

    static const uint8_t payload[4] = {1, 2, 3, 4};
    (void)Usb::submit_in(1, std::span<const uint8_t>{payload});
    (void)Usb::submit_out(2, 64);
    (void)Usb::out_data(2);
    (void)Usb::setup();
    (void)Usb::take_events();

    (void)Usb::frame();
    (void)Usb::address();
    (void)Usb::pulled_up();
    (void)Usb::buffer_used();
    (void)Usb::buffer_free();
    (void)Usb::errors();
    (void)Usb::overruns();

    Usb::deconfigure_endpoints();
    Usb::connect(false);
    Usb::release();
}

void usb_stack_verbs() {
    constexpr Serial serial;

    Device::start();
    (void)Device::state();
    (void)Device::configured();
    (void)Device::address();
    (void)Device::resets();
    (void)Device::setups();
    (void)Device::stalls();
    (void)Device::suspends();
    (void)Device::resumes();
    (void)Device::last_request();

    (void)serial.write_byte('x');
    uint8_t b = 0;
    (void)Serial::read_byte(b);
    (void)Serial::tx_idle();
    (void)Serial::dtr();
    Serial::reset();

    Device::isr();
    Device::stop();
}

extern "C" BRIO_CH32_INTERRUPT void usb_lp_can1_rx0_handler() { Device::isr(); }
