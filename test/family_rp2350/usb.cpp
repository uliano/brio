// USB family smoke TU: every verb of the device controller
// (rp2350/usb.hpp over datasheet 12.7), the `UsbController` contract it
// realizes, the stack and the CDC ACM class instantiated over it, and
// the compile-time rules - erratum RP2350-E12's clock margin, the
// endpoint bit arithmetic of the four registers laid out per endpoint
// and direction, and the DPRAM map. Compiled for both packages and both
// architectures: this block is bonded on every package, so what the four
// builds prove is that one source serves both instruction sets.
#include <span>

#include "rp2350/usb.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

#include "rp2350/platform.hpp"

using namespace brio;

// ---- the contract ------------------------------------------------------------

static_assert(UsbController<Usb>);

// ---- the two regions and the map of 12.7.3.7.2 -------------------------------

static_assert(usbctrl_dpram_base == 0x5010'0000UL);
static_assert(usbctrl_regs_base == 0x5011'0000UL);
static_assert(Usb::dpram_bytes == 4096u);
static_assert(Usb::ep0_buffer == 0x100u);
static_assert(Usb::buffers_start == 0x180u);
static_assert(Usb::endpoint_count == 16u);
static_assert(Usb::max_packet == usb_full_speed_packet);
// 3840 bytes of buffer space from 0x100, which is the sixty 64-byte
// buffers of 12.7.1.1.1 - two of them endpoint zero's own pair, so what
// this driver has to hand out past 0x180 is fifty-eight.
static_assert((Usb::dpram_bytes - Usb::ep0_buffer) == 3840u);
static_assert((Usb::dpram_bytes - Usb::ep0_buffer) / 64u == 60u);
static_assert((Usb::dpram_bytes - Usb::buffers_start) / 64u == 58u);
// The bulk OUT endpoint takes two of them, so a CDC port costs four.
static_assert(Usb::out_slots == 2u);
static_assert(Usb::irq() == USBCTRL_IRQ_IRQn && Usb::irq_line == USBCTRL_IRQ_IRQn);

// ---- the four registers laid out per endpoint and direction --------------------

// BUFF_STATUS, BUFF_CPU_SHOULD_HANDLE, EP_ABORT and EP_STATUS_STALL_NAK
// all carry two bits per endpoint, THE IN DIRECTION EVEN and the OUT
// direction odd, which is the arithmetic take_events() and the abort
// verbs do by hand.
static_assert(USB_BUFF_STATUS_EP0_IN_BITS == (1u << 0) && USB_BUFF_STATUS_EP0_OUT_BITS == (1u << 1));
static_assert(USB_BUFF_STATUS_EP1_IN_BITS == (1u << 2) && USB_BUFF_STATUS_EP1_OUT_BITS == (1u << 3));
static_assert(USB_BUFF_STATUS_EP15_IN_BITS == (1u << 30) && USB_BUFF_STATUS_EP15_OUT_BITS == (1u << 31));
static_assert(USB_EP_ABORT_EP1_IN_BITS == USB_BUFF_STATUS_EP1_IN_BITS);
static_assert(USB_EP_ABORT_DONE_EP1_IN_BITS == USB_BUFF_STATUS_EP1_IN_BITS);
static_assert(USB_EP_STATUS_STALL_NAK_EP1_IN_BITS == USB_BUFF_STATUS_EP1_IN_BITS);
// The error registers are two bits an endpoint too, but the pair is one
// direction each: a count going up on the transmit side, two flags on
// the receive side.
static_assert(USB_EP_TX_ERROR_EP0_BITS == 0x3u && USB_EP_TX_ERROR_EP1_BITS == 0xCu);
static_assert(USB_EP_RX_ERROR_EP0_TRANSACTION_BITS == (1u << 0) && USB_EP_RX_ERROR_EP0_SEQ_BITS == (1u << 1));

// ---- the two pads, which are bank 1's and not bank 0's -------------------------

// DP and DM are GPIO 56 and 57 of BANK 1 (9.4's table 647), and their
// function select - the IO_QSPI block's, not IO_BANK0's - resets to
// NULL. Nothing of this stratum writes it (pin.hpp is bank 0 alone), so
// with USB_MUXING written whole by init() the pads are the PHY's.
static_assert(IO_QSPI_USBPHY_DP_CTRL_FUNCSEL_RESET == IO_QSPI_USBPHY_DP_CTRL_FUNCSEL_VALUE_NULL);
static_assert(IO_QSPI_USBPHY_DM_CTRL_FUNCSEL_RESET == IO_QSPI_USBPHY_DM_CTRL_FUNCSEL_VALUE_NULL);
static_assert(IO_QSPI_USBPHY_DP_CTRL_FUNCSEL_VALUE_SIO_56 == 5u);
static_assert(IO_QSPI_USBPHY_DM_CTRL_FUNCSEL_VALUE_SIO_57 == 5u);

// ---- erratum RP2350-E12's margin ---------------------------------------------

// clk_sys at least ten per cent above clk_usb: 52.8 MHz is the edge, and
// the rule is an inequality and not a table.
static_assert(!usb_clk_sys_ok(12'000'000UL));
static_assert(!usb_clk_sys_ok(48'000'000UL));
static_assert(!usb_clk_sys_ok(52'799'999UL));
static_assert(usb_clk_sys_ok(52'800'000UL));
static_assert(usb_clk_sys_ok(125'000'000UL));
static_assert(usb_clk_sys_ok(150'000'000UL));
// The rate the board runs at, and the one the RP2040 ran at, both pass.
static_assert(usb_clk_sys_ok(Clock<ClockSource::pll, 150'000'000UL>::hz));

// ---- the watchdog configuration ------------------------------------------------

constexpr UsbDeviceWatchdogConfig watchdog{.limit = 48'000u, .reset_on_fire = true};
static_assert(watchdog.limit == 48'000u && watchdog.reset_on_fire);
static_assert(UsbDeviceWatchdogConfig{}.limit == 0u && UsbDeviceWatchdogConfig{}.reset_on_fire);

// ---- the per-endpoint error record ----------------------------------------------

static_assert(!UsbEndpointErrors{}.any());
static_assert(UsbEndpointErrors{.tx_count = 1}.any());
static_assert(UsbEndpointErrors{.rx_transaction = true}.any());
static_assert(UsbEndpointErrors{.rx_sequence = true}.any());

// ---- the stack and the class over this controller ---------------------------------

using P = Rp2350Platform<>;
using Port = UsbCdcAcm<Usb, P>;

struct Descriptors {
    static constexpr auto device =
        usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count), Port::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto product = usb_string_descriptor("brio");
    static std::span<const uint8_t> string(uint8_t index) {
        switch (index) {
        case 0: return language;
        case 1: return product;
        default: return {};
        }
    }
};
using Device = UsbDevice<Usb, Descriptors, Port>;

static_assert(UsbClass<Port>);
static_assert(UsbDeviceDescriptors<Descriptors>);
static_assert(Port::interface_count == 2u);
static_assert(Descriptors::device.size() == 18u);
static_assert(Descriptors::configuration.size() == 67u);
// The class asks the controller how many OUT packets it may arm, and
// this controller answers with its two halves.
static_assert(Port::out_slots() == 2u);

// ---- every verb ---------------------------------------------------------------------

void usb_verbs() {
    constexpr Clock<ClockSource::pll, 150'000'000UL> clock;
    (void)Usb::init(clock);
    Usb::release();

    // the controller contract
    Usb::connect(true);
    Usb::connect(false);
    Usb::set_address(7);
    (void)Usb::configure_endpoint(usb_ep_in(1), UsbEndpointType::interrupt, 8);
    (void)Usb::configure_endpoint(usb_ep_out(2), UsbEndpointType::bulk, 64);
    (void)Usb::configure_endpoint(usb_ep_in(2), UsbEndpointType::bulk, 64);
    (void)Usb::configure_endpoint(usb_ep_out(3), UsbEndpointType::isochronous, 64);
    Usb::deconfigure_endpoints();
    Usb::stall(usb_ep_in(0), true);
    Usb::stall(usb_ep_out(0), false);
    (void)Usb::stalled(usb_ep_in(1));
    const uint8_t payload[4] = {1, 2, 3, 4};
    (void)Usb::submit_in(0, std::span<const uint8_t>(payload, 4));
    (void)Usb::submit_in(0, {});
    (void)Usb::submit_out(2, 64);
    (void)Usb::out_data(2);
    (void)Usb::setup();
    (void)Usb::take_events();

    // the raw register faces
    volatile uint32_t& address_register = Usb::regs().ADDR_ENDP;
    volatile uint32_t& setup_low = Usb::dpram_word(0);
    volatile uint32_t& control_word = Usb::ep_control(1, true);
    volatile uint32_t& buffer_word = Usb::buf_control(0, false);
    const uint32_t read_back = address_register | setup_low | control_word | buffer_word;
    (void)read_back;
    (void)Usb::dpram_bytes_at(Usb::ep0_buffer);

    // the readbacks
    (void)Usb::connected();
    (void)Usb::suspended();
    (void)Usb::vbus_detected();
    (void)Usb::pulled_up();
    (void)Usb::address();
    (void)Usb::frame();
    (void)Usb::sie_status();
    (void)Usb::line_state();
    (void)Usb::take_errors();
    (void)Usb::buffers_used();

    // the PHY's isolation, this chip's one new duty
    (void)Usb::phy_isolated();
    Usb::phy_isolate(true);
    Usb::phy_isolate(false);
    (void)Usb::controller_enabled();
    (void)Usb::phy_as_gpio();
    (void)Usb::muxing();

    // LINESTATE_TUNING, read and never written
    (void)Usb::linestate_tuning();
    (void)Usb::buffer_control_double_read_fix();
    (void)Usb::wake_on_any_bus_activity();

    // the diagnostics of 12.7.2.2.4
    (void)Usb::endpoint_errors(0);
    (void)Usb::endpoint_errors(15);
    (void)Usb::endpoint_errors(16);   // out of range: an empty record, never a read past the field
    Usb::clear_endpoint_errors();
    (void)Usb::take_endpoint_error();
    (void)Usb::take_short_packet();
    Usb::ep0_stop_on_short_packet(true);
    Usb::ep0_stop_on_short_packet(false);
    (void)Usb::sof_timestamp_raw();
    (void)Usb::sof_timestamp_last();
    (void)Usb::since_sof();
    (void)Usb::sm_state();
    (void)Usb::sm_main();
    (void)Usb::sm_bus_control();
    (void)Usb::sm_rx_deserialiser();
    (void)Usb::arm_device_watchdog(watchdog);
    (void)Usb::arm_device_watchdog({.limit = 0x4'0000u});   // past 18 bits: refused
    (void)Usb::device_watchdog_armed();
    (void)Usb::device_watchdog_limit();
    (void)Usb::take_device_watchdog_fired();
    Usb::disarm_device_watchdog();

    // the abort path
    Usb::abort(usb_ep_out(2), true);
    (void)Usb::aborted(usb_ep_out(2));
    (void)Usb::abort_done(usb_ep_out(2));
    Usb::abort(usb_ep_out(2), false);

    // the stall and NAK reporting
    (void)Usb::stall_nak_status();
    Usb::clear_stall_nak(0xFFFF'FFFFu);
    Usb::ep0_report(true, false);
    Usb::ep0_report(false, false);
    (void)Usb::buff_cpu_should_handle();
    (void)Usb::buff_status();

    // the remote wakeup
    Usb::remote_wakeup();

    // the interrupt surface
    Usb::interrupt_enable(USB_INTS_BUFF_STATUS_BITS);
    (void)Usb::interrupt_enable();
    (void)Usb::interrupt_raw();
    (void)Usb::interrupt_status();
    Usb::force_interrupt(0);
    (void)Usb::forced_interrupts();
}

// ---- the stack's verbs over it ---------------------------------------------------

void usb_stack_verbs() {
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
    Device::stop();

    (void)Port::write_byte('x');
    uint8_t b = 0;
    (void)Port::read_byte(b);
    (void)Port::write(&b, 1);
    (void)Port::tx_idle();
    (void)Port::take_rx_edge();
    (void)Port::configured();
    (void)Port::dtr();
    (void)Port::rts();
    (void)Port::line_coding();
    (void)Port::coding_changes();
    (void)Port::line_state_changes();
    (void)Port::breaks();
    (void)Port::rx_bytes();
    (void)Port::tx_bytes();
    (void)Port::rx_overruns();
    (void)Port::in_armed();
    (void)Port::out_armed();
}
