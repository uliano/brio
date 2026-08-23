// USART family smoke TU: every package must compile this
// (instantiation only). The USART is the driver with the widest
// per-package spread of the whole target - three instances on 28/32
// pins, five on 48, six on 64, and route tables whose ALT1 halves
// appear one signal at a time as the package grows. What this TU
// proves is that the resource and every task instantiate for the
// instances and routes THIS package has, and that the arithmetic and
// the route table agree with the device header.
#include "avrdx/clock.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/usart.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 24'000'000>;

// ---- the route table against the device header's route enums --------------

// USART0 is whole on every package: PA0-PA3 and PA4-PA7.
static_assert(usart_route_exists(0, UsartRoute::def));
static_assert(usart_route_exists(0, UsartRoute::alt1));
static_assert(usart_pin(0, UsartRoute::def, UsartSignal::txd).port == 'A');
static_assert(usart_pin(0, UsartRoute::def, UsartSignal::txd).pin == 0);
static_assert(usart_pin(0, UsartRoute::alt1, UsartSignal::xdir).pin == 7);
static_assert(usart_pin(0, UsartRoute::alt1, UsartSignal::xdir).bonded);

// A pinless route has no pins at all - that is what makes it pinless.
static_assert(!usart_pin(0, UsartRoute::none, UsartSignal::txd).bonded);

// USART2's ALT1 has no XCK position on any package (17.5.9).
static_assert(!usart_pin(2, UsartRoute::alt1, UsartSignal::xck).bonded);

#if defined(USART5)
// 64 pins: everything exists, ALT1 included.
static_assert(usart_count == 6);
static_assert(usart_package_pins == 64);
static_assert(usart_route_exists(5, UsartRoute::alt1));
static_assert(usart_pin(5, UsartRoute::alt1, UsartSignal::txd).port == 'G');
static_assert(usart_pin(3, UsartRoute::alt1, UsartSignal::xck).bonded);   // PB6
static_assert(usart_pin(4, UsartRoute::alt1, UsartSignal::txd).bonded);   // PE4
#elif defined(USART3)
// 48 pins: USART0..4; PORTB stops at PB5 and PORTE at PE3.
static_assert(usart_count == 5);
static_assert(usart_package_pins == 48);
static_assert(usart_route_exists(3, UsartRoute::alt1));                   // PB4/PB5
static_assert(!usart_pin(3, UsartRoute::alt1, UsartSignal::xck).bonded);  // no PB6
static_assert(!usart_pin(3, UsartRoute::alt1, UsartSignal::xdir).bonded); // no PB7
static_assert(!usart_route_exists(4, UsartRoute::alt1));                  // no PE4..PE7
static_assert(usart_pin(4, UsartRoute::def, UsartSignal::xdir).bonded);   // PE3
#else
// 28/32 pins: USART0..2 only, and PORTC stops at PC3.
static_assert(usart_count == 3);
static_assert(!usart_route_exists(1, UsartRoute::alt1));
#if defined(TWI1)
static_assert(usart_package_pins == 32);
static_assert(usart_route_exists(2, UsartRoute::alt1));                   // PF4/PF5
static_assert(usart_pin(2, UsartRoute::def, UsartSignal::xck).bonded);    // PF2
#else
static_assert(usart_package_pins == 28);
static_assert(!usart_route_exists(2, UsartRoute::alt1));                  // no PF4/PF5
static_assert(!usart_pin(2, UsartRoute::def, UsartSignal::xck).bonded);   // no PF2
#endif
#endif

// ---- the baud arithmetic (27.3.2.2.1) -------------------------------------

// 24 MHz, 115200, normal speed: 64 * 24e6 / (16 * 115200) = 833.33 -> 833.
static_assert(usart_baud_reg(24'000'000u, 115'200u) == 833);
// Double speed halves the divisor.
static_assert(usart_baud_reg(24'000'000u, 115'200u, 8) == 1667);
// Synchronous: only BAUD[15:6], the fraction zero.
static_assert(usart_baud_reg(24'000'000u, 1'000'000u, 2) == (12u << 6));
static_assert((usart_baud_reg(24'000'000u, 1'000'000u, 2) & 0x3Fu) == 0);
// Out of the register's range in both directions.
static_assert(usart_baud_reg(24'000'000u, 24'000'000u) == 0);   // BAUD would be 4
static_assert(usart_baud_reg(24'000'000u, 1u) == 0);            // BAUD would not fit 16 bits
static_assert(usart_min_clock_hz(115'200u) == 115'200u * 16u);
static_assert(usart_min_clock_hz(115'200u, 8) == 115'200u * 8u);
// The readback of what the generator settled on.
static_assert(usart_actual_baud(24'000'000u, 833) == 115'246u);
static_assert(usart_actual_baud(24'000'000u, 12u << 6, 2) == 1'000'000u);
static_assert(usart_samples(UsartMode::async, UsartRxMode::normal) == 16);
static_assert(usart_samples(UsartMode::async, UsartRxMode::clk2x) == 8);
static_assert(usart_samples(UsartMode::sync) == 2);
static_assert(usart_samples(UsartMode::mspi) == 2);

// Character sizes.
static_assert(usart_data_bits(UsartBits::five) == 5);
static_assert(usart_data_bits(UsartBits::nine_high_first) == 9);
static_assert(usart_is_9bit(UsartBits::nine_low_first));
static_assert(!usart_is_9bit(UsartBits::eight));

// The legality rule a package enforces: no XCK, no synchronous mode.
static_assert(usart_config_valid<0>(UsartConfig{.mode = UsartMode::sync, .baud = 64}));
static_assert(!usart_config_valid<0>(UsartConfig{.route = UsartRoute::none,
                                                 .mode = UsartMode::sync, .baud = 64}));
static_assert(!usart_config_valid<0>(UsartConfig{.mode = UsartMode::sync,
                                                 .rx_mode = UsartRxMode::clk2x, .baud = 64}));
static_assert(usart_config_valid<0>(UsartConfig{.baud = 64, .rs485 = true}));

// ---- the resource, instance by instance -----------------------------------

/// Everything Usart<n> can do, for one instance and one route, only
/// compiled when this package has both (a plain `if` would instantiate
/// the missing instance and kill the TU).
template <uint8_t n, UsartRoute route>
void usart_resource_route() {
    if constexpr (n < usart_count && usart_route_exists(n, route)) {
        using U = Usart<n>;
        (void)U::init({.route = route, .baud = usart_baud_reg(24'000'000u, 115'200u)});
        (void)U::init({.route = route, .bits = UsartBits::nine_low_first,
                       .parity = UsartParity::even, .two_stop = true,
                       .rx_mode = UsartRxMode::clk2x,
                       .baud = usart_baud_reg(24'000'000u, 115'200u, 8),
                       .multiprocessor = true, .debug_run = true});
        (void)U::init({.route = route, .mode = UsartMode::ircom,
                       .baud = usart_baud_reg(24'000'000u, 9600u),
                       .irda_event_input = true, .tx_pulse = 0xFF, .rx_pulse = 3});
        (void)U::init({.route = route, .rx_mode = UsartRxMode::linauto,
                       .baud = usart_baud_reg(24'000'000u, 19200u),
                       .ab_window = UsartAbWindow::wdw3});
        if constexpr (route != UsartRoute::none) {
            (void)U::init({.route = route, .baud = usart_baud_reg(24'000'000u, 9600u),
                           .loop_back = true, .open_drain = true});
            if constexpr (usart_pin(n, route, UsartSignal::xck).bonded) {
                U::template init<UsartConfig{.route = route, .mode = UsartMode::sync,
                                             .baud = 64u << 6}>();
                U::template init<UsartConfig{.route = route, .mode = UsartMode::mspi,
                                             .baud = 64u << 6, .lsb_first = true,
                                             .sample_trailing = true}>();
                U::template init<UsartConfig{.route = route, .mode = UsartMode::sync,
                                             .baud = 64u << 6, .sync_client = true}>();
            }
            if constexpr (usart_pin(n, route, UsartSignal::xdir).bonded) {
                U::template init<UsartConfig{.route = route, .baud = 833, .rs485 = true}>();
            }
        }
        U::release();
    }
}

template <uint8_t n>
void usart_resource() {
    if constexpr (n < usart_count) {
        using U = Usart<n>;
        usart_resource_route<n, UsartRoute::none>();
        usart_resource_route<n, UsartRoute::def>();
        usart_resource_route<n, UsartRoute::alt1>();

        U::enable_rx(true);
        U::enable_tx(false);
        (void)U::rx_enabled();
        (void)U::tx_enabled();
        U::disable();
        U::flush_rx();
        U::frame({.bits = UsartBits::seven, .parity = UsartParity::odd, .two_stop = true});
        (void)U::bits();
        (void)U::mode();
        (void)U::baud_reg();
        U::baud_reg(1234);
        (void)U::set_baud(24'000'000u, 460'800u);
        (void)U::actual_baud(24'000'000u);
        (void)U::samples();
        (void)U::max_host_xck_hz(24'000'000u);
        (void)U::max_client_xck_hz(24'000'000u);
        U::rx_mode(UsartRxMode::genauto);
        (void)U::rx_mode();
        U::multiprocessor(true);
        U::open_drain(false);
        U::loop_back(true);
        U::rs485(false);
        U::arm_start_of_frame();
        (void)U::start_of_frame_armed();
        U::disarm_start_of_frame();
        U::auto_baud_window(UsartAbWindow::wdw2);
        (void)U::auto_baud_window();
        U::irda_event_input(true);
        U::irda_on(EventChannel<0>{});
        U::tx_pulse(0xFF);
        (void)U::tx_pulse();
        U::rx_pulse(7);
        (void)U::rx_pulse();
        U::debug_run(true);
        (void)U::status();
        (void)U::rxc_flag();
        (void)U::dre_flag();
        (void)U::txc_flag();
        (void)U::rxs_flag();
        (void)U::isf_flag();
        (void)U::break_flag();
        U::clear_txc();
        U::clear_rxs();
        U::clear_isf();
        U::clear_break();
        U::wait_for_break();
        U::recover_from_isf();
        U::enable_rxc_interrupt(true);
        U::enable_txc_interrupt(true);
        U::enable_dre_interrupt(true);
        U::enable_rxs_interrupt(true);
        U::enable_autobaud_error_interrupt(true);
        (void)U::receive();
        (void)U::template receive_as<UsartBits::eight>();
        (void)U::template receive_as<UsartBits::nine_low_first>();
        (void)U::template receive_as<UsartBits::nine_high_first>();
        U::transmit(0x1AA);
        U::template transmit_as<UsartBits::eight>(0x55);
        U::template transmit_as<UsartBits::nine_low_first>(0x1AA);
        U::template transmit_as<UsartBits::nine_high_first>(0x1AA);
        (void)U::send(0x55, 10);
        (void)U::poll();
        (void)U::wait(10);
        (void)U::wait_line_idle(10);
        (void)U::route();
        (void)U::routed();
        (void)U::has_route(UsartRoute::alt1);
        (void)U::txd(UsartRoute::def);
        (void)U::rxd(UsartRoute::def);
        (void)U::xck(UsartRoute::def);
        (void)U::xdir(UsartRoute::def);
        EventChannel<0>::source(typename U::XckEvent{});
        U::release();
    }
}

// ---- the tasks -------------------------------------------------------------

/// The interrupt-driven transport, on every instance this package has.
template <int n>
void usart_transport() {
    if constexpr (n < static_cast<int>(usart_count)) {
        using T = Uart<n, UsartRoute::def>;
        constexpr SysClock clock;
        T::init(clock, 115'200u);
        T::rebase(12'000'000u);
        (void)T::can_baud(24'000'000u, 460'800u);
        (void)T::min_hz_for(460'800u);
        (void)T::actual_baud(24'000'000u);
        (void)T::rxc();
        T::dre();
        (void)T::write_byte('x');
        uint8_t b = 0;
        (void)T::read_byte(b);
        (void)T::write(&b, 1);
        (void)T::rx_pending();
        (void)T::tx_idle();
        (void)T::rx_overruns();
        (void)T::frame_errors();
        (void)T::parity_errors();
        (void)T::hw_overruns();
        T::clear_errors();
        static_assert(ByteTransport<T>);
    }
}

/// Every other task, on the instance/route pairs this package bonds.
template <uint8_t n, UsartRoute route>
void usart_tasks() {
    if constexpr (n < usart_count && usart_route_exists(n, route)) {
        constexpr SysClock clock;
        if constexpr (OneWire<n, route>::available) {
            using W = OneWire<n, route>;
            (void)W::init(clock, 9600u);
            (void)W::init(clock, 9600u, {.bits = UsartBits::nine_high_first,
                                         .parity = UsartParity::odd, .two_stop = true});
            W::talk();
            (void)W::listen(10);
            (void)W::send(0x5A, 10);
            (void)W::echo_matches(0x5A, 10);
            (void)W::line();
            W::rebase(12'000'000u);
            W::release();
            static_assert(ClockUser<W>);
        }
        if constexpr (Rs485<n, route>::available) {
            using R = Rs485<n, route>;
            (void)R::init(clock, 19'200u);
            (void)R::init(clock, 19'200u, {}, /*one_wire=*/true);
            (void)R::drive_enable();
            static_assert(R::guard_bits == 1);
            R::rebase(12'000'000u);
            R::release();
            static_assert(ClockUser<R>);
        }
        if constexpr (SyncHost<n, route>::available) {
            using H = SyncHost<n, route>;
            (void)H::init(clock, 1'000'000u);
            H::invert_xck(true);
            (void)H::clock_pin();
            H::rebase(12'000'000u);
            static_assert(ClockUser<H>);

            using C = SyncClient<n, route>;
            (void)C::init();
            (void)C::max_xck_hz(24'000'000u);
            (void)C::clock_pin();

            using M = MspiHost<n, route>;
            (void)M::init(clock, 4'000'000u);
            (void)M::init(clock, 4'000'000u, {.lsb_first = true, .sample_trailing = true,
                                              .invert_sck = true});
            (void)M::transfer(0xA5, 10);
            M::rebase(12'000'000u);
            static_assert(ClockUser<M>);
        }
        if constexpr (IrdaLink<n, route>::available) {
            using I = IrdaLink<n, route>;
            (void)I::init(clock, 9600u);
            (void)I::init(clock, 115'200u, 0xFF, 5);
            (void)I::init(clock, 1'000'000u);          // refused at run time: above IrDA's ceiling
            I::receive_on(EventChannel<0>{});
            I::rebase(12'000'000u);
            static_assert(ClockUser<I>);
        }
        if constexpr (AutoBaud<n, route>::available) {
            using A = AutoBaud<n, route>;
            (void)A::init(clock, 19'200u);
            (void)A::init(clock, 19'200u, A::Kind::lin, UsartAbWindow::wdw1,
                          {.parity = UsartParity::even});
            A::arm_break();
            (void)A::break_detected();
            A::clear_break();
            (void)A::sync_error();
            A::recover();
            (void)A::measured_baud_reg();
            (void)A::measured_baud(24'000'000u);
            (void)A::poll();
            A::rebase(12'000'000u);
            A::release();
            static_assert(ClockUser<A>);
        }
    }
}

void usart_family() {
    usart_resource<0>(); usart_resource<1>(); usart_resource<2>();
    usart_resource<3>(); usart_resource<4>(); usart_resource<5>();

    usart_transport<0>(); usart_transport<1>(); usart_transport<2>();
    usart_transport<3>(); usart_transport<4>(); usart_transport<5>();

    usart_tasks<0, UsartRoute::def>(); usart_tasks<0, UsartRoute::alt1>();
    usart_tasks<1, UsartRoute::def>(); usart_tasks<1, UsartRoute::alt1>();
    usart_tasks<2, UsartRoute::def>(); usart_tasks<2, UsartRoute::alt1>();
    usart_tasks<3, UsartRoute::def>(); usart_tasks<3, UsartRoute::alt1>();
    usart_tasks<4, UsartRoute::def>(); usart_tasks<4, UsartRoute::alt1>();
    usart_tasks<5, UsartRoute::def>(); usart_tasks<5, UsartRoute::alt1>();
}
