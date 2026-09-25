// mcu: ch32v203f8 ch32v203g8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// USBFS family smoke TU: the host/device controller of RM ch. 23 in
// device mode as util/usb's UsbController, the whole stack composed over
// it - the control-endpoint machine and a CDC ACM port - and every verb
// of the driver, the wake-up line's body and the vector bindings a
// program writes.
//
// Nine parts of the thirteen: the CH32V203 carries this block where its
// datasheet's table 2-1 counts a USBHD (the F8, the G8, the C6, the C8
// and the RB, on PB6/PB7) and every CH32V303 carries it on PA11/PA12;
// the F6, the G6, the K6 and the K8 have none, and a neg TU proves the
// refusal there.
#include "ch32vx03/clock.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usbfs.hpp"
#include "util/stream.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

#include <span>

using namespace brio;

static_assert(device::has_usbfs);
static_assert(pad_bonded(Pad{device::usbfs_dm_port, device::usbfs_dm_pin}) &&
              pad_bonded(Pad{device::usbfs_dp_port, device::usbfs_dp_pin}));
// The two pads are one port's neighbours on every part that has them.
static_assert(device::usbfs_dm_port == device::usbfs_dp_port &&
              device::usbfs_dp_pin == device::usbfs_dm_pin + 1u);

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;
using Usb = Usbfs<>;
using Cdc = Usbfs<256>;   // exactly what a CDC ACM port spends
using Serial = UsbCdcAcm<Usb, P>;

// The claim the driver makes, once more where the fixture can see it.
static_assert(UsbController<Usb> && UsbController<Cdc>);
static_assert(UsbClass<Serial>);
static_assert(ByteSink<Serial> && ByteSource<Serial>);

// The block's shape: eight endpoint numbers, 64-byte packets, endpoint
// zero's one buffer, and the pool that no set of claims exhausts.
static_assert(Usb::endpoint_count == 8 && Usb::max_packet == 64);
static_assert(usbfs_ep0_bytes == 64 && usbfs_half_bytes == 64);
static_assert(Usb::pool_size == usbfs_pool_all && usbfs_pool_all == 960);
static_assert(Cdc::pool_size == 256);
// The wake-up line: table 9-3's 20 on the CH32V203, and 18 on the
// CH32V30x_D8, where a host's resume was measured raising 18 alone.
static_assert(Usb::wakeup_line == (device::device_class == DeviceClass::v30x_d8
                                       ? Exti::line_usbd_wakeup
                                       : Exti::line_usbfs_wakeup) &&
              Exti::implemented(Usb::wakeup_line));
static_assert(Usb::wakes_on_line18 == (device::device_class == DeviceClass::v30x_d8));
static_assert(irq_exists(Usb::wakeup_irq) && Exti::irq(Usb::wakeup_line) == Usb::wakeup_irq);
static_assert(usbfs_min_hclk_hz == 12'000'000UL);

// The mode nibbles of the four mode bytes, in the chapter's order.
static_assert(usbfs_mod_slot(1).reg == 0 && usbfs_mod_slot(1).shift == 4);
static_assert(usbfs_mod_slot(4).reg == 0 && usbfs_mod_slot(4).shift == 0);
static_assert(usbfs_mod_slot(2).reg == 1 && usbfs_mod_slot(2).shift == 0);
static_assert(usbfs_mod_slot(3).reg == 1 && usbfs_mod_slot(3).shift == 4);
static_assert(usbfs_mod_slot(5).reg == 2 && usbfs_mod_slot(6).reg == 2 && usbfs_mod_slot(6).shift == 4);
static_assert(usbfs_mod_slot(7).reg == 3 && usbfs_mod_slot(0).reg == 0xFF && usbfs_mod_slot(8).reg == 0xFF);

// The bus floor this driver carries, and a tree that clears it; a neg TU
// proves the refusal below it.
static_assert(SysClock::usb_hz == usb_required_hz && SysClock::hz >= usbfs_min_hclk_hz);

// Under a dynamic clock the same two questions are asked of the rate in
// force, at run time: a pack that includes a rate with no 48 MHz compiles,
// and init() answers false there.
using Low = Clock<ClockSource::internal, 8'000'000>;
using Paced = DynamicClock<Rates<SysClock, Low>>;
static_assert(!Paced::is_static && Paced::rate_usb_hz(0) == usb_required_hz && Paced::rate_usb_hz(1) == 0u);

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

void usbfs_verbs() {
    constexpr SysClock clock;
    (void)Usb::init(clock);
    (void)Cdc::init(clock);
    (void)Usb::init(Paced{});

    Usb::connect(true);
    Usb::set_address(7);
    (void)Usb::configure_endpoint(0x81, UsbEndpointType::interrupt, 8);
    (void)Usb::configure_endpoint(0x02, UsbEndpointType::bulk, 64);
    (void)Usb::configure_endpoint<usb_ep_in(2), UsbEndpointType::bulk, 64>();
    Usb::stall(0x81, true);
    (void)Usb::stalled(0x81);
    Usb::stall(0x81, false);

    static const uint8_t payload[4] = {1, 2, 3, 4};
    (void)Usb::submit_in(1, std::span<const uint8_t>{payload});
    (void)Usb::submit_out(2, 64);
    (void)Usb::out_data(2);
    (void)Usb::setup();
    (void)Usb::take_events();

    (void)Usb::arm_wakeup(true);
    (void)Usb::wakeup_armed();
    (void)Usb::wakeups();
    (void)Usb::arm_wakeup(false);
    Usb::count_frames(true);
    (void)Usb::frames();
    (void)Usb::frame();
    Usb::count_frames(false);

    (void)Usb::regs();
    (void)Usb::address();
    (void)Usb::pulled_up();
    (void)Usb::attached();
    (void)Usb::suspended();
    (void)Usb::bus_in_reset();
    (void)Usb::sie_free();
    (void)Usb::dp_high();
    (void)Usb::dm_high();
    (void)Usb::flags();
    (void)Usb::status();
    (void)Usb::setup_length();
    (void)Usb::setup_status();
    (void)Usb::buffer_used();
    (void)Usb::buffer_free();
    (void)Usb::errors();
    (void)Usb::overruns();
    (void)Usb::toggle_errors();
    (void)Usb::received(2);
    (void)Usb::in_toggle(2);
    (void)Usb::out_toggle(2);
    (void)Usb::set_out_toggle(2, true);
    Usb::auto_pause(true);
    (void)Usb::auto_pause();

    Usb::deconfigure_endpoints();
    Usb::connect(false);
    Usb::release();
    Cdc::release();
}

void usbfs_stack_verbs() {
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

// The vectors a program binds: the controller's, whose body is the
// stack's, and the wake-up line's - vector 58 on the CH32V303, 60 on the
// CH32V203 (Usb::wakeup_irq).
extern "C" BRIO_CH32_INTERRUPT void usbfs_handler() { Device::isr(); }
extern "C" BRIO_CH32_INTERRUPT void usb_wakeup_handler() {
    if constexpr (Usb::wakes_on_line18) {
        (void)Usb::wakeup_isr();
    }
}
extern "C" BRIO_CH32_INTERRUPT void usbfs_wakeup_handler() {
    if constexpr (!Usb::wakes_on_line18) {
        (void)Usb::wakeup_isr();
    }
}
