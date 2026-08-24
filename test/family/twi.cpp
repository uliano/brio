// TWI family smoke TU: every package must compile this (instantiation
// only). TWI0 is everywhere, TWI1 arrives at 32 pins; what varies is
// the ROUTE table and, above all, which routes bond a DUAL pin pair -
// the pair a Dual mode client needs.
#include "avrdx/clock.hpp"
#include "avrdx/twi.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 24'000'000>;

// ---- the route table against the device header's route enums --------------

// TWI0 has all three routes on every package, and no pinless one.
static_assert(twi_route_exists(0, TwiRoute::def));
static_assert(twi_route_exists(0, TwiRoute::alt1));
static_assert(twi_route_exists(0, TwiRoute::alt2));
static_assert(twi_pin(0, TwiRoute::def, TwiSignal::sda).port == 'A');
static_assert(twi_pin(0, TwiRoute::def, TwiSignal::sda).pin == 2);
static_assert(twi_pin(0, TwiRoute::def, TwiSignal::scl).pin == 3);
static_assert(twi_pin(0, TwiRoute::alt1, TwiSignal::sda).port == 'A');   // same pair as DEFAULT
static_assert(twi_pin(0, TwiRoute::alt2, TwiSignal::sda).port == 'C');
static_assert(twi_pin(0, TwiRoute::alt2, TwiSignal::scl).pin == 3);
// TWI0 DEFAULT always has its dual pair: PC2/PC3.
static_assert(twi_has_dual(0, TwiRoute::def));
static_assert(twi_dual_pin(0, TwiRoute::def, TwiSignal::sda).port == 'C');
static_assert(twi_dual_pin(0, TwiRoute::def, TwiSignal::sda).pin == 2);

// ---- the baud arithmetic (29.3.2.2.1) ------------------------------------

static_assert(twi_scl_hz(TwiSpeed::standard_100k) == 100'000u);
static_assert(twi_scl_hz(TwiSpeed::fast_plus_1m) == 1'000'000u);
static_assert(twi_low_min_ns(TwiSpeed::standard_100k) == 4700u);
static_assert(twi_rise_budget_ns(TwiSpeed::fast_400k) == 300u);
static_assert(twi_needs_fm_plus(TwiSpeed::fast_plus_1m));
static_assert(!twi_needs_fm_plus(TwiSpeed::fast_400k));

// At 24 MHz, with the specification's worst-case rise AND fall charged
// to the bus, all three speeds are decided by the tLOW FLOOR (equation
// 29-5): equation 29-3 alone would give 103 / 22 / 6.
static_assert(twi_fall_budget_ns(TwiSpeed::standard_100k) == 300u);
static_assert(twi_fall_budget_ns(TwiSpeed::fast_plus_1m) == 120u);
static_assert(*twi_baud_for(24'000'000u, TwiSpeed::standard_100k) == 115);
static_assert(*twi_baud_for(24'000'000u, TwiSpeed::fast_400k) == 34);
static_assert(*twi_baud_for(24'000'000u, TwiSpeed::fast_plus_1m) == 10);
// ... and the tLOW they produce clears the mode's floor WITH the fall
// time subtracted (ticks at 24 MHz -> nanoseconds: x 1000 / 24).
static_assert(twi_low_ticks(115) * 1000ul / 24ul - 300ul >= 4700ul);
static_assert(twi_low_ticks(34) * 1000ul / 24ul - 300ul >= 1300ul);
static_assert(twi_low_ticks(10) * 1000ul / 24ul - 120ul >= 500ul);
// A bus that DECLARES its (stiffer) real timing gets a faster clock and
// still meets the floor: 4.79 us of low at 100 kHz.
static_assert(*twi_baud_for(24'000'000u, TwiSpeed::standard_100k, 200u, 150u) == 113);
static_assert(twi_low_ticks(113) * 1000ul / 24ul - 150ul >= 4700ul);
// The period floor (tR = 0) and the period the rise budget predicts.
static_assert(twi_period_ticks(24'000'000u, 115, 0) == 240u);
static_assert(twi_period_ticks(24'000'000u, 115, 1000u) == 264u);
static_assert(twi_scl_hz_at(24'000'000u, 115, 1000u) == 90'909u);
// A 1 MHz bus at BAUD 10 is 30 ticks minimum: CLK_PER/30 at 24 MHz.
static_assert(twi_period_ticks(24'000'000u, 10, 0) == 30u);
// A clock the divider cannot span: BAUD would have to exceed 255.
static_assert(!twi_baud_for(64'000'000u, TwiSpeed::standard_100k).has_value());
static_assert(twi_clock_ok(24'000'000u, TwiSpeed::fast_plus_1m));
static_assert(!twi_clock_ok(2'000'000u, TwiSpeed::fast_plus_1m));

// ---- the SDAHOLD swap (DA errata 2.14.2) ---------------------------------

// The enum names TRUE nanoseconds on both families; only the encoding
// moves. OFF and 500 ns are never swapped, and the swap is its own
// inverse.
static_assert(twi_sdahold_code(TwiSdaHold::off) == 0);
static_assert(twi_sdahold_code(TwiSdaHold::ns500) == 3);
static_assert(twi_sdahold_of(twi_sdahold_code(TwiSdaHold::ns50)) == TwiSdaHold::ns50);
static_assert(twi_sdahold_of(twi_sdahold_code(TwiSdaHold::ns300)) == TwiSdaHold::ns300);
#if defined(MVIO)
static_assert(twi_sdahold_code(TwiSdaHold::ns50) == 1);    // DB: straight
static_assert(twi_sdahold_code(TwiSdaHold::ns300) == 2);
#else
static_assert(twi_sdahold_code(TwiSdaHold::ns50) == 2);    // DA: swapped
static_assert(twi_sdahold_code(TwiSdaHold::ns300) == 1);
#endif

// ---- configuration legality ----------------------------------------------

static_assert(twi_config_valid<0>({.route = TwiRoute::def}));
// Fast-mode Plus without its pads is refused.
static_assert(!twi_config_valid<0>({.speed = TwiSpeed::fast_plus_1m}));
static_assert(twi_config_valid<0>({.fm_plus = true, .speed = TwiSpeed::fast_plus_1m}));
// An instance with neither half enabled does nothing.
static_assert(!twi_config_valid<0>({.host = false}));
// Dual mode needs a client and a bonded dual pair. TWI0 DEFAULT always
// has PC2/PC3, so this one is legal on every package.
static_assert(twi_config_valid<0>({.route = TwiRoute::def, .client = true, .dual = true}));
static_assert(!twi_config_valid<0>({.host = true, .dual = true}));   // no client
// Addresses are seven bits.
static_assert(!twi_config_valid<0>({.client = true, .address = 0x80}));

// ---- per-package route and dual tables -----------------------------------

#if defined(PORTG)
// 64 pins: every route of both instances, and every dual pair.
static_assert(twi_package_pins == 64);
static_assert(twi_instance_count == 2);
static_assert(twi_route_exists(1, TwiRoute::alt2));
static_assert(twi_has_dual(0, TwiRoute::alt1));                       // PC6/PC7
static_assert(twi_dual_pin(0, TwiRoute::alt2, TwiSignal::scl).pin == 7);
static_assert(twi_has_dual(1, TwiRoute::def));                        // PB2/PB3
static_assert(twi_has_dual(1, TwiRoute::alt1));                       // PB6/PB7
static_assert(twi_dual_pin(1, TwiRoute::alt1, TwiSignal::sda).port == 'B');
static_assert(twi_dual_pin(1, TwiRoute::alt1, TwiSignal::sda).pin == 6);
static_assert(twi_has_dual(1, TwiRoute::alt2));
static_assert(twi_pin(1, TwiRoute::alt2, TwiSignal::sda).port == 'B');
void use_64() {
    (void)TwiHost<1, TwiRoute::alt2>::init(SysClock{});
    (void)TwiClient<1, TwiRoute::alt1, true>::init(SysClock{}, {.address = 0x40});
}
#elif defined(PORTE)
// 48 pins: TWI1 has all three routes; PB6/PB7 do not exist, so TWI1
// ALT1/ALT2 have no dual pair here.
static_assert(twi_package_pins == 48);
static_assert(twi_instance_count == 2);
static_assert(twi_route_exists(1, TwiRoute::alt2));
static_assert(twi_has_dual(0, TwiRoute::alt1));                       // PC6/PC7
static_assert(twi_has_dual(1, TwiRoute::def));                        // PB2/PB3
static_assert(!twi_has_dual(1, TwiRoute::alt1));                      // no PB6/PB7
static_assert(!twi_has_dual(1, TwiRoute::alt2));
static_assert(twi_pin(1, TwiRoute::alt2, TwiSignal::sda).port == 'B');
static_assert(twi_pin(1, TwiRoute::alt2, TwiSignal::sda).bonded);
void use_48() {
    (void)TwiHost<1, TwiRoute::alt2>::init(SysClock{});
    (void)TwiClient<0, TwiRoute::alt1, true>::init(SysClock{}, {.address = 0x40});
    (void)TwiClient<1, TwiRoute::def, true>::init(SysClock{}, {.address = 0x41});
}
#elif defined(TWI1)
// 32 pins: TWI1 exists with DEFAULT and ALT1 only, and NO route of
// either instance bonds a dual pair except TWI0 DEFAULT.
static_assert(twi_package_pins == 32);
static_assert(twi_instance_count == 2);
static_assert(twi_route_exists(1, TwiRoute::def));
static_assert(twi_route_exists(1, TwiRoute::alt1));
static_assert(!twi_route_exists(1, TwiRoute::alt2));                  // no PB2/PB3
static_assert(!twi_has_dual(0, TwiRoute::alt1));                      // no PC6/PC7
static_assert(!twi_has_dual(1, TwiRoute::def));                       // no PORTB at all
void use_32() {
    (void)TwiHost<1, TwiRoute::alt1>::init(SysClock{});
    (void)TwiClient<0, TwiRoute::def, true>::init(SysClock{}, {.address = 0x40});
}
#else
// 28 pins: TWI0 only, and only its DEFAULT route has a dual pair.
static_assert(twi_package_pins == 28);
static_assert(twi_instance_count == 1);
static_assert(!twi_route_exists(1, TwiRoute::def));
static_assert(!twi_has_dual(0, TwiRoute::alt1));
static_assert(!twi_has_dual(0, TwiRoute::alt2));
void use_28() {
    (void)TwiHost<0, TwiRoute::alt2>::init(SysClock{});
    (void)TwiClient<0, TwiRoute::def, true>::init(SysClock{}, {.address = 0x40});
}
#endif

// ---- the resource and the tasks on what every package has ----------------

using T0 = Twi<0>;
using Host0 = TwiHost<0>;
using Client0 = TwiClient<0>;
using DualClient0 = TwiClient<0, TwiRoute::def, true>;

static_assert(Host0::available);
static_assert(Client0::available);
static_assert(DualClient0::available);
static_assert(DualClient0::sda_pin().port == 'C');

uint8_t tx_buf[4];
uint8_t rx_buf[4];

void use_resource() {
    (void)T0::init<TwiConfig{.route = TwiRoute::def, .sda_hold = TwiSdaHold::ns300,
                             .speed = TwiSpeed::fast_400k,
                             .rise_ns = 200, .fall_ns = 150,
                             .timeout = TwiTimeout::us100}>(24'000'000u);
    (void)T0::init({.route = TwiRoute::alt2, .fm_plus = true,
                    .speed = TwiSpeed::fast_plus_1m, .client = true,
                    .address = 0x42, .general_call = true}, 24'000'000u);
    // CTRLA
    T0::input_level(TwiInputLevel::smbus);
    T0::sda_setup(TwiSdaSetup::cycles8);
    T0::sda_hold(TwiSdaHold::ns50);
    (void)T0::sda_hold();
    (void)T0::sda_hold_code();
    T0::fm_plus(false);
    (void)T0::fm_plus();
    (void)T0::input_level();
    (void)T0::sda_setup();
    // DUALCTRL / DBGCTRL
    (void)T0::dual_mode(true, TwiInputLevel::i2c, TwiSdaHold::ns500, false);
    (void)T0::dual_mode();
    T0::debug_run(true);
    (void)T0::debug_run();
    // host
    T0::host_enable(true);
    (void)T0::host_enabled();
    T0::enable_read_interrupt(true);
    T0::enable_write_interrupt(true);
    (void)T0::host_interrupts();
    T0::quick_command(true);
    (void)T0::quick_command();
    T0::timeout(TwiTimeout::us200);
    (void)T0::timeout();
    T0::host_smart(true);
    (void)T0::host_smart();
    T0::host_command(TwiHostCmd::recv_trans, TwiAck::nack);
    T0::ack_action(TwiAck::ack);
    T0::recover();
    (void)T0::host_status();
    (void)T0::read_flag();
    (void)T0::write_flag();
    (void)T0::clock_hold();
    (void)T0::rx_nack();
    (void)T0::arbitration_lost();
    (void)T0::bus_error();
    (void)T0::bus_state();
    T0::clear_host_flags(TWI_RIF_bm | TWI_WIF_bm);
    T0::force_idle();
    T0::set_baud(60);
    (void)T0::baud();
    T0::address_write(0x60);
    T0::address_read(0x60);
    T0::maddr(0xC1);
    (void)T0::maddr();
    T0::host_write(0x5A);
    (void)T0::host_read();
    // client
    T0::client_enable(true);
    (void)T0::client_enabled();
    T0::enable_data_interrupt(true);
    T0::enable_address_interrupt(true);
    T0::enable_stop_interrupt(true);
    (void)T0::client_interrupts();
    T0::promiscuous(true);
    (void)T0::promiscuous();
    T0::client_smart(true);
    (void)T0::client_smart();
    T0::client_command(TwiClientCmd::response, TwiAck::ack);
    T0::client_ack_action(TwiAck::nack);
    (void)T0::client_status();
    (void)T0::data_flag();
    (void)T0::address_flag();
    (void)T0::client_clock_hold();
    (void)T0::client_rx_nack();
    (void)T0::collision();
    (void)T0::client_bus_error();
    (void)T0::host_reading();
    (void)T0::address_match();
    T0::clear_client_flags(TWI_DIF_bm | TWI_APIF_bm);
    T0::client_address(0x42, true);
    (void)T0::client_address();
    (void)T0::general_call();
    T0::address_mask(0x07);
    T0::second_address(0x44);
    (void)T0::addrmask_field();
    (void)T0::second_address_enabled();
    T0::client_write(0xA5);
    (void)T0::client_read();
    // ISR bodies, clock and teardown
    (void)T0::take_host().nack();
    (void)T0::take_client().is_stop();
    (void)T0::clk_per_hz();
    T0::note_clock(12'000'000u);
    T0::bus_timing(250, 200);
    (void)T0::rise_ns();
    (void)T0::fall_ns();
    (void)T0::clock_ok(TwiSpeed::fast_400k);
    (void)T0::set_speed(TwiSpeed::standard_100k);
    (void)T0::speed();
    (void)T0::actual_scl_hz(0);
    (void)T0::rebase(24'000'000u);
    (void)T0::routed();
    (void)T0::route();
    T0::release();
}

void use_tasks() {
    (void)Host0::init(SysClock{}, {.speed = TwiSpeed::fast_400k,
                                   .rise_ns = 200, .fall_ns = 150,
                                   .timeout = TwiTimeout::us100,
                                   .sda_hold = TwiSdaHold::ns300,
                                   .smart = true});
    (void)Host0::start(typename Host0::Request{
        0x60, tx_buf, 2, rx_buf, 4, {}, TwiSpeed::standard_100k});
    (void)Host0::isr();
    (void)Host0::status();
    Host0::rebase(12'000'000u);
    (void)Host0::actual_scl_hz(300u);
    (void)Host0::baud();
    (void)Host0::speed();
    (void)Host0::clock_ok();
    Host0::quick_command(true);
    (void)Host0::quick_command();
    Host0::recover();
    (void)Host0::bus_state();
    Host0::release();

    (void)Client0::init(SysClock{}, {.address = 0x42, .general_call = true,
                                     .address_mask = 0x03, .smart = true,
                                     .stop_interrupt = true});
    (void)Client0::addressed();
    (void)Client0::data_ready();
    (void)Client0::host_reading();
    (void)Client0::matched_address();
    (void)Client0::last_address();
    Client0::respond(TwiAck::ack);
    Client0::complete(TwiAck::nack);
    (void)Client0::receive(TwiAck::nack);
    Client0::transmit(0x5A);
    (void)Client0::nacked();
    (void)Client0::collision();
    Client0::clear_collision();
    (void)Client0::bus_error();
    Client0::clear_bus_error();
    (void)Client0::wait_address(10);
    (void)Client0::wait_data(10);
    Client0::enable_data_interrupt(true);
    Client0::enable_address_interrupt(true);
    Client0::enable_stop_interrupt(true);
    Client0::address(0x44);
    Client0::address_mask(0x01);
    Client0::second_address(0x45);
    Client0::promiscuous(true);
    (void)Client0::isr().collision();
    Client0::rebase(24'000'000u);
    (void)Client0::max_scl_hz();
    (void)Client0::can_follow(400'000u);
    Client0::release();

    (void)DualClient0::init(SysClock{}, {.address = 0x50});
    DualClient0::release();
}
