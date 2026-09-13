// USB OTG device-mode family smoke TU (RM0383 ch. 22, RM0390 ch. 31,
// RM0090 ch. 34 and 35). What varies across the pack is not the
// programmer's model - it is one Synopsys core at one or two base
// addresses - but WHICH CORES a part carries and which of the three
// GCCFG generations it has, so that is what this TU pins down, twice
// over where two independent header symbols can be made to agree.
//
// A family fixture is the one place that may ask the header a question
// TWICE, by two independent symbols, and assert the two answers agree.
#include "stm32f4/device_tables.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/usb.hpp"
#include "util/usb/cdc.hpp"

using namespace brio;

// ---- the reserve's own answers, on every part ---------------------------------

// A core is present exactly when it has a vector, and the vector is a
// different symbol in the pack from the base address.
static_assert(otg_present(OtgCore::fs) == (otg_irq(OtgCore::fs) != NonMaskableInt_IRQn),
              "the OTG_FS core and its global interrupt come and go together");
static_assert(otg_present(OtgCore::hs) == (otg_irq(OtgCore::hs) != NonMaskableInt_IRQn),
              "the OTG_HS core and its global interrupt come and go together");
static_assert(otg_present(OtgCore::fs) == (otg_wakeup_irq(OtgCore::fs) != NonMaskableInt_IRQn));
static_assert(otg_present(OtgCore::hs) == (otg_wakeup_irq(OtgCore::hs) != NonMaskableInt_IRQn));

// ... and exactly when exti.hpp's reserve derives its wake-up line, which
// is a third derivation of the same fact.
static_assert(otg_present(OtgCore::fs) == exti_line_implemented(otg_wakeup_exti_line(OtgCore::fs)));
static_assert(otg_present(OtgCore::hs) == exti_line_implemented(otg_wakeup_exti_line(OtgCore::hs)));

// A part with any core has a GCCFG generation, and a part with none has
// none: the VBUS style is derived from bit names, the presence from base
// addresses, and the two must agree.
static_assert((otg_vbus_style() != OtgVbusStyle::none) ==
                  (otg_present(OtgCore::fs) || otg_present(OtgCore::hs)),
              "GCCFG's bits exist exactly where an OTG core does");

// The session-valid status bit is named on exactly the parts that have a
// core, whichever of its two spellings the header uses.
static_assert((otg_session_valid_bit() != 0u) ==
              (otg_present(OtgCore::fs) || otg_present(OtgCore::hs)));

// The override pair belongs to the later GCCFG generation and to no
// other: the two derivations are independent symbols.
static_assert(otg_has_session_override() == (otg_vbus_style() == OtgVbusStyle::vbus_detect));
static_assert((otg_session_override_bits() != 0u) == otg_has_session_override());

// The gates: the FS core's on AHB2, the HS core's on AHB1, on every part
// that has them.
static_assert(!otg_facts(OtgCore::fs).gate_on_ahb1);
static_assert(otg_facts(OtgCore::hs).gate_on_ahb1 == otg_present(OtgCore::hs));
static_assert((otg_facts(OtgCore::fs).clock_mask != 0u) == otg_present(OtgCore::fs));
static_assert((otg_facts(OtgCore::hs).ulpi_clock_mask != 0u) == otg_present(OtgCore::hs));

// The manuals' constants, carried per core and not per part.
static_assert(!otg_present(OtgCore::fs) ||
              (otg_facts(OtgCore::fs).fifo_words == 320u && otg_facts(OtgCore::fs).endpoints == 4u));
static_assert(!otg_present(OtgCore::hs) ||
              (otg_facts(OtgCore::hs).fifo_words == 1024u &&
               otg_facts(OtgCore::hs).endpoints == 6u));

// The pads the datasheets bond, and the PHY select only the HS core needs.
static_assert(!otg_present(OtgCore::fs) ||
              (otg_facts(OtgCore::fs).dm_port == 'A' && otg_facts(OtgCore::fs).dm_pin == 11u &&
               otg_facts(OtgCore::fs).dp_port == 'A' && otg_facts(OtgCore::fs).dp_pin == 12u &&
               otg_facts(OtgCore::fs).pad_function == 10u &&
               !otg_facts(OtgCore::fs).phy_select_writable));
static_assert(!otg_present(OtgCore::hs) ||
              (otg_facts(OtgCore::hs).dm_port == 'B' && otg_facts(OtgCore::hs).dm_pin == 14u &&
               otg_facts(OtgCore::hs).dp_port == 'B' && otg_facts(OtgCore::hs).dp_pin == 15u &&
               otg_facts(OtgCore::hs).pad_function == 12u &&
               otg_facts(OtgCore::hs).phy_select_writable));
static_assert(!otg_present(OtgCore::fs) || gpio_port_present(otg_facts(OtgCore::fs).dm_port));
static_assert(!otg_present(OtgCore::hs) || gpio_port_present(otg_facts(OtgCore::hs).dm_port));

// Table 132, end to end: the fastest band, every boundary, and the floor
// RM0383 22.3.3's caution states - below which no value serves.
static_assert(otg_turnaround_for(180'000'000) == 0x6);
static_assert(otg_turnaround_for(32'000'000) == 0x6);
static_assert(otg_turnaround_for(31'999'999) == 0x7);
static_assert(otg_turnaround_for(27'500'000) == 0x7);
static_assert(otg_turnaround_for(24'000'000) == 0x8);
static_assert(otg_turnaround_for(21'800'000) == 0x9);
static_assert(otg_turnaround_for(20'000'000) == 0xA);
static_assert(otg_turnaround_for(18'500'000) == 0xB);
static_assert(otg_turnaround_for(17'200'000) == 0xC);
static_assert(otg_turnaround_for(16'000'000) == 0xD);
static_assert(otg_turnaround_for(15'000'000) == 0xE);
static_assert(otg_turnaround_for(14'200'000) == 0xF);
static_assert(otg_turnaround_for(14'199'999) == 0);
static_assert(otg_turnaround_for(0) == 0);

// The transceiver is powered up whatever the part's GCCFG generation, and
// the two answers differ exactly in the VBUS bit.
static_assert(!otg_present(OtgCore::fs) ||
              (otg_gccfg_device(false) != 0u && otg_gccfg_device(true) != 0u &&
               otg_gccfg_device(false) != otg_gccfg_device(true)));

// ---- the driver, where the part has a core -------------------------------------

#if defined(USB_OTG_FS_PERIPH_BASE) || defined(USB_OTG_HS_PERIPH_BASE)

// THE CLOCK THIS FIXTURE PICKS, and why it is picked from the reserve
// and not from a device name. 96 MHz from an 8 MHz root gives the PLL's
// Q output 48 MHz exactly and sits under every ladder the reserve knows
// (100 MHz on the F411 class, 168 and 180 on the others) - but a part
// whose reference manual was not read has NO ladder, and stm32f4/clock.hpp
// refuses every rate above the 16 MHz reset one there. So the fixture
// takes the PLL clock where the ladder is known and the reset clock where
// it is not, and the init() calls below are compiled only in the first
// case: a part with no ladder is a part this driver cannot be clocked on,
// which is a property of the stratum and not a hole in this TU.
template <bool ladder_known>
struct UsbFixtureClock {
    using type = Clock<ClockSource::pll_hse, 96'000'000, 8'000'000>;
};
template <>
struct UsbFixtureClock<false> {
    using type = Clock<ClockSource::hsi, hsi_hz>;
};
using SysClock = UsbFixtureClock<sysclk_ladder().known>::type;
static_assert(sysclk_ladder().known == (SysClock::usb_hz == otg_clock_hz),
              "a known ladder is exactly what lets this family reach an exact 48 MHz");

template <typename Usb>
void exercise() {
    if constexpr (SysClock::usb_hz == otg_clock_hz) {
        constexpr SysClock clock;
        (void)Usb::init(clock);
        (void)Usb::init(clock, true);   // the VBUS pad kept and sensed
    }
    Usb::release();

    // the UsbController contract, every verb
    Usb::connect(true);
    Usb::connect(false);
    Usb::set_address(37);
    (void)Usb::configure_endpoint(usb_ep_in(1), UsbEndpointType::interrupt, 8);
    (void)Usb::configure_endpoint(usb_ep_out(2), UsbEndpointType::bulk, 64);
    (void)Usb::configure_endpoint(usb_ep_in(2), UsbEndpointType::bulk, 64);
    Usb::deconfigure_endpoints();
    Usb::stall(usb_ep_in(0), true);
    Usb::stall(usb_ep_out(0), false);
    (void)Usb::stalled(usb_ep_in(0));
    const uint8_t bytes[4] = {1, 2, 3, 4};
    (void)Usb::submit_in(0, std::span<const uint8_t>(bytes, 4));
    (void)Usb::submit_in(0, {});
    (void)Usb::submit_out(0, 64);
    (void)Usb::submit_out(2, 64);
    (void)Usb::out_data(2);
    (void)Usb::setup();
    (void)Usb::take_events();

    // the readbacks
    (void)Usb::address();
    (void)Usb::pulled_up();
    (void)Usb::suspended();
    (void)Usb::erratic_error();
    (void)Usb::enumerated_speed();
    (void)Usb::frame();
    (void)Usb::session_valid();
    (void)Usb::in_device_mode();
    (void)Usb::core_id();
    (void)Usb::fifo_words_used();
    (void)Usb::rx_entries();
    (void)Usb::fifo_waits();
    (void)Usb::stalls();
    (void)Usb::timeouts();
    (void)Usb::setup_overruns();
    (void)Usb::early_suspends();
    (void)Usb::mode_mismatches();
    (void)Usb::sessions();
    (void)Usb::otg_events();
    (void)Usb::enumerations();
    (void)Usb::out_armed(2);

    // the low-power verbs of 22.8
    Usb::gate_clocks(true);
    (void)Usb::phy_suspended();
    Usb::gate_clocks(false);
    Usb::remote_wakeup(true);
    Usb::remote_wakeup(false);

    // the budget, stated and checked at compile time
    static_assert(Usb::rx_words + Usb::ep0_tx_words <= Usb::fifo_words);
    static_assert(Usb::tx_words_for(8) == 16, "the register's minimum is 16 words");
    static_assert(Usb::tx_words_for(64) == 16);
    static_assert(Usb::max_packet == usb_full_speed_packet);
    static_assert(Usb::out_slots >= 1);
}

void f() {
#if defined(USB_OTG_FS_PERIPH_BASE)
    exercise<UsbFs>();
    static_assert(UsbFs::endpoint_count == 4);
    static_assert(UsbFs::fifo_words == 320);
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
    exercise<UsbHs>();
    static_assert(UsbHs::endpoint_count == 6);
    static_assert(UsbHs::fifo_words == 1024);
#endif
}

// The CDC ACM class over this controller: the stack's own types
// instantiated so the family check covers them too, the way the RP2040's
// fixture does.
#if defined(USB_OTG_FS_PERIPH_BASE)
using CdcPort = UsbCdcAcm<UsbFs, Stm32f4Platform<>, 0, 1, 2>;
struct Descriptors {
    static constexpr auto device =
        usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration = usb_concat(
        usb_configuration_head(9 + CdcPort::descriptor_bytes, CdcPort::interface_count), CdcPort::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static std::span<const uint8_t> string(uint8_t index) {
        return index == 0u ? std::span<const uint8_t>(language) : std::span<const uint8_t>{};
    }
};
using Device = UsbDevice<UsbFs, Descriptors, CdcPort>;

void g() {
    Device::start();
    Device::isr();
    Device::stop();
    (void)Device::configured();
    (void)CdcPort::write_byte('x');
    uint8_t b = 0;
    (void)CdcPort::read_byte(b);
    (void)CdcPort::tx_idle();
    (void)CdcPort::out_slots();
    static_assert(CdcPort::out_slots() == UsbFs::out_slots,
                  "the class arms as many OUT packets as this controller takes at once");
}
#endif

#endif   // the part has an OTG core at all
