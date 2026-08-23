// SPI family smoke TU: every package must compile this (instantiation
// only). Both instances exist on every package of the family, so what
// varies here is the ROUTE table - and one route is refused not by the
// device header but by an erratum (SPI1 ALT2 on 48 pins).
#include "avrdx/clock.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/spi.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 24'000'000>;

// ---- the route table against the device header's route enums --------------

// The default routes exist on every package: SPI0 PA4-PA7, SPI1 PC0-PC3.
static_assert(spi_route_exists(0, SpiRoute::def));
static_assert(spi_route_exists(1, SpiRoute::def));
static_assert(spi_route_exists(0, SpiRoute::none));
static_assert(spi_route_exists(1, SpiRoute::none));
static_assert(spi_pin(0, SpiRoute::def, SpiSignal::mosi).port == 'A');
static_assert(spi_pin(0, SpiRoute::def, SpiSignal::mosi).pin == 4);
static_assert(spi_pin(0, SpiRoute::def, SpiSignal::ss).pin == 7);
static_assert(spi_pin(1, SpiRoute::def, SpiSignal::sck).port == 'C');
static_assert(spi_pin(1, SpiRoute::def, SpiSignal::sck).pin == 2);

// A pinless route has no pins at all - that is what makes it pinless.
static_assert(!spi_pin(0, SpiRoute::none, SpiSignal::sck).bonded);

// ---- the rate table (28.5.1) ---------------------------------------------

static_assert(spi_division(SpiClock::div2) == 2);
static_assert(spi_division(SpiClock::div128) == 128);
static_assert(spi_presc_bits(SpiClock::div2) == (SPI_PRESC_DIV4_gc | SPI_CLK2X_bm));
static_assert(spi_presc_bits(SpiClock::div64) == SPI_PRESC_DIV64_gc);
static_assert(spi_clock_of(spi_presc_bits(SpiClock::div32)) == SpiClock::div32);
static_assert(spi_sck_hz(24'000'000u, SpiClock::div4) == 6'000'000u);
// The chooser takes the FASTEST division that stays under the ceiling.
static_assert(*spi_clock_for(24'000'000u, 6'000'000u) == SpiClock::div4);
static_assert(*spi_clock_for(24'000'000u, 5'999'999u) == SpiClock::div8);
static_assert(*spi_clock_for(24'000'000u, 200'000u) == SpiClock::div128);
static_assert(!spi_clock_for(24'000'000u, 100'000u).has_value());
static_assert(spi_max_host_sck_hz(24'000'000u) == 12'000'000u);
static_assert(spi_max_client_sck_hz(24'000'000u) == 4'000'000u);
static_assert(spi_cpol(SpiMode::mode2) && spi_cpol(SpiMode::mode3));
static_assert(!spi_cpol(SpiMode::mode1) && spi_cpha(SpiMode::mode1));

// ---- configuration legality ----------------------------------------------

// A pinless host must disable Client Select (DA errata 2.10.1).
static_assert(spi_config_valid<0>({.route = SpiRoute::none, .client_select_disable = true}));
static_assert(!spi_config_valid<0>({.route = SpiRoute::none, .client_select_disable = false}));
// A pinless client has nothing to be clocked by.
static_assert(!spi_config_valid<0>({.route = SpiRoute::none, .role = SpiRole::client}));
// BUFWR without BUFEN means nothing.
static_assert(!spi_config_valid<0>({.role = SpiRole::client, .buffer_wait = true}));
static_assert(spi_config_valid<0>({.role = SpiRole::client, .buffer_mode = true,
                                   .buffer_wait = true}));

// ---- per-package route tables --------------------------------------------

#if defined(PORTG)
// 64 pins: every route of both instances, ALT2 included.
static_assert(spi_package_pins == 64);
static_assert(spi_route_exists(0, SpiRoute::alt1));
static_assert(spi_route_exists(0, SpiRoute::alt2));
static_assert(spi_route_exists(1, SpiRoute::alt1));
static_assert(spi_route_exists(1, SpiRoute::alt2));
static_assert(spi_pin(0, SpiRoute::alt1, SpiSignal::mosi).port == 'E');
static_assert(spi_pin(0, SpiRoute::alt2, SpiSignal::mosi).port == 'G');
static_assert(spi_pin(0, SpiRoute::alt2, SpiSignal::ss).pin == 7);
static_assert(spi_pin(1, SpiRoute::alt1, SpiSignal::ss).pin == 7);      // PC7
static_assert(spi_pin(1, SpiRoute::alt2, SpiSignal::sck).port == 'B');  // PB6
static_assert(spi_pin(1, SpiRoute::alt2, SpiSignal::sck).bonded);
void use_64() {
    (void)SpiHost<0, SpiRoute::alt2>::init(SysClock{});
    (void)SpiClient<1, SpiRoute::alt2>::init();
}
#elif defined(PORTE)
// 48 pins: ALT1 on both instances; SPI0 ALT2 (PORTG) does not exist and
// SPI1 ALT2 is refused - the header bonds PB4/PB5 only and DB errata
// 2.11.1 declares the position non-functional.
static_assert(spi_package_pins == 48);
static_assert(spi_route_exists(0, SpiRoute::alt1));
static_assert(!spi_route_exists(0, SpiRoute::alt2));
static_assert(spi_route_exists(1, SpiRoute::alt1));
static_assert(!spi_route_exists(1, SpiRoute::alt2));
static_assert(spi_pin(0, SpiRoute::alt1, SpiSignal::sck).port == 'E');
static_assert(spi_pin(0, SpiRoute::alt1, SpiSignal::sck).pin == 2);
static_assert(spi_pin(0, SpiRoute::alt1, SpiSignal::ss).bonded);        // PE3
static_assert(!spi_pin(1, SpiRoute::alt2, SpiSignal::sck).bonded);      // no PB6 here
void use_48() {
    (void)SpiHost<0, SpiRoute::alt1>::init(SysClock{}, 2'000'000u);
    (void)SpiClient<0, SpiRoute::alt1>::init();
    (void)SpiHost<1, SpiRoute::alt1>::init(SysClock{});
}
#else
// 28/32 pins: DEFAULT and NONE only, for both instances.
static_assert(spi_package_pins <= 32);
static_assert(!spi_route_exists(0, SpiRoute::alt1));
static_assert(!spi_route_exists(0, SpiRoute::alt2));
static_assert(!spi_route_exists(1, SpiRoute::alt1));
static_assert(!spi_route_exists(1, SpiRoute::alt2));
void use_small() {
    (void)SpiHost<1>::init(SysClock{});
    (void)SpiClient<1>::init();
}
#endif

// ---- the resource and the tasks on what every package has ----------------

using S0 = Spi<0>;
using S1 = Spi<1>;
using Host0 = SpiHost<0>;                       // DEFAULT route
using Pinless = SpiHost<0, SpiRoute::none>;     // a host with no pins at all
using Client1 = SpiClient<1>;

static_assert(Host0::available);
static_assert(Pinless::available);
static_assert(Client1::available);
static_assert(!SpiClient<0, SpiRoute::none>::available);

void use_resource() {
    (void)S0::init<SpiConfig{.route = SpiRoute::def, .clock = SpiClock::div32}>();
    (void)S0::init({.route = SpiRoute::none, .clock = SpiClock::div2});
    (void)S1::init<SpiConfig{.role = SpiRole::client, .mode = SpiMode::mode3,
                             .buffer_mode = true, .buffer_wait = true}>();
    S0::enable(false);
    S0::clock(SpiClock::div8);
    S0::mode(SpiMode::mode2);
    S0::lsb_first(true);
    S0::client_select_disable(false);
    S0::buffer_mode(true, true);
    S0::write(0x5A);
    (void)S0::read();
    (void)S0::poll();
    (void)S0::transfer(0xFF, 100);
    (void)S0::if_flag();
    (void)S0::write_collision();
    S0::clear_if();
    S0::clear_flags_by_data_access();
    (void)S0::rxc_flag();
    (void)S0::txc_flag();
    (void)S0::dre_flag();
    (void)S0::ss_flag();
    (void)S0::overflow_flag();
    S0::clear_rxc();
    S0::clear_txc();
    S0::clear_ss_flag();
    S0::clear_overflow();
    S0::enable_interrupt(true);
    S0::enable_rxc_interrupt(true);
    S0::enable_txc_interrupt(true);
    S0::enable_dre_interrupt(true);
    S0::enable_ss_interrupt(true);
    (void)S0::take_normal().data;
    (void)S0::take_buffer();
    (void)S0::is_host();
    (void)S0::demoted();
    S0::restore_host();
    (void)S0::sck_hz(24'000'000u);
    (void)S0::routed();
    S0::release();
    S1::release();
    // The SCK event generator is legal on every channel.
    static_assert(S0::SckEvent::legal_on(0));
}

uint8_t rx_buf[4];

void use_tasks() {
    (void)Host0::init(SysClock{});
    (void)Pinless::init(SysClock{});
    (void)Host0::start(typename Host0::Request{
        PinRef{}, PinRef{}, nullptr, 0, nullptr, rx_buf, 4, {},
        SpiClock::div64, SpiMode::mode3, true, 10});
    (void)Host0::isr();
    Host0::rebase(12'000'000u);
    (void)Host0::max_sck_hz();
    (void)Host0::ceiling_clock();
    (void)Host0::sck_hz(SpiClock::div4);
    Host0::release();

    (void)Client1::init({.mode = SpiMode::mode1, .buffer_mode = true});
    (void)Client1::selected();
    Client1::preload(0xA5);
    (void)Client1::poll();
    (void)Client1::exchange(0x5A, 100);
    (void)Client1::ready_for_data();
    (void)Client1::transfer_complete();
    (void)Client1::overflow();
    (void)Client1::write_collision();
    Client1::enable_rxc_interrupt(true);
    (void)Client1::take_normal().complete();
    (void)Client1::take_buffer();
    Client1::rebase(24'000'000u);
    (void)Client1::max_sck_hz();
    (void)Client1::can_follow(1'000'000u);
    Client1::release();
}
