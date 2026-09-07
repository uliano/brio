// I2C family smoke TU: the I2c<n> resource, both tasks above it and the
// timing arithmetic between them - on I2C1 and I2C2, which every G0 has,
// and on I2C3 where the header declares one (the G0B1/G0C1 class). The
// pads are the Nucleo-G0B1RE self-link's (DS13560 tables 15, 13): I2C1
// PB8/PB9 at AF6 and I2C2 PA11/PA12 at AF6, both on ports every package
// of the family carries.
#include "stm32g0/clock.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/i2c.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 64'000'000>;

// ---- the reserve ------------------------------------------------------------

static_assert(i2c_present(1) && i2c_present(2));
static_assert(!i2c_present(0) && !i2c_present(4));
static_assert(i2c_bus_clock(1) == RCC_APBENR1_I2C1EN);
static_assert(i2c_bus_clock(2) == RCC_APBENR1_I2C2EN);
static_assert(i2c_irq(1) == I2C1_IRQn);
static_assert(i2c_dma_rx_request(1) == 10 && i2c_dma_tx_request(1) == 11);
static_assert(i2c_dma_rx_request(2) == 12 && i2c_dma_tx_request(2) == 13);
static_assert(i2c_dma_rx_request(3) == 62 && i2c_dma_tx_request(3) == 63);
// I2C1 has all three of table 165's conditional rows on every part.
static_assert(i2c_has_independent_clock(1) && i2c_has_smbus(1) &&
              i2c_wakes_from_stop(1));
static_assert(i2c_clock_select_pos(1) == RCC_CCIPR_I2C1SEL_Pos);
static_assert(i2c_exti_line(1) == 23);
// I2C3 has none of them anywhere, present or not.
static_assert(!i2c_has_independent_clock(3) && !i2c_has_smbus(3) &&
              !i2c_wakes_from_stop(3));
static_assert(i2c_clock_select_pos(3) == 0xFF);
static_assert(i2c_exti_line(3) == 0xFF);
// The per-pad and per-instance fast-mode-plus bits.
static_assert(i2c_pad_fmp_bit('B', 8) == SYSCFG_CFGR1_I2C_PB8_FMP);
static_assert(i2c_pad_fmp_bit('B', 9) == SYSCFG_CFGR1_I2C_PB9_FMP);
static_assert(i2c_pad_fmp_bit('B', 6) == SYSCFG_CFGR1_I2C_PB6_FMP);
static_assert(i2c_pad_fmp_bit('A', 9) == SYSCFG_CFGR1_I2C_PA9_FMP);
// PA11 and PA12 - THIS BENCH'S OWN I2C2 PADS - have no bit of their own.
static_assert(i2c_pad_fmp_bit('A', 11) == 0);
static_assert(i2c_pad_fmp_bit('A', 12) == 0);
static_assert(i2c_pad_fmp_bit('C', 0) == 0);
static_assert(i2c_instance_fmp_bit(1) == SYSCFG_CFGR1_I2C1_FMP);
static_assert(i2c_instance_fmp_bit(2) == SYSCFG_CFGR1_I2C2_FMP);

#if defined(I2C3_BASE)
static_assert(i2c_present(3));
static_assert(i2c_bus_clock(3) == RCC_APBENR1_I2C3EN);
static_assert(i2c_irq(2) == I2C2_3_IRQn && i2c_irq(3) == I2C2_3_IRQn);
// The G0B1/G0C1 class is exactly where I2C2 gains the three rows.
static_assert(i2c_has_independent_clock(2) && i2c_has_smbus(2) &&
              i2c_wakes_from_stop(2));
static_assert(i2c_clock_select_pos(2) == RCC_CCIPR_I2C2SEL_Pos);
static_assert(i2c_exti_line(2) == 22);
static_assert(i2c_instance_fmp_bit(3) == SYSCFG_CFGR1_I2C3_FMP);
#else
static_assert(!i2c_present(3));
static_assert(i2c_irq(2) == I2C2_IRQn);
static_assert(!i2c_has_independent_clock(2) && !i2c_has_smbus(2) &&
              !i2c_wakes_from_stop(2));
static_assert(i2c_clock_select_pos(2) == 0xFF);
static_assert(i2c_exti_line(2) == 0xFF);
static_assert(i2c_instance_fmp_bit(3) == 0);
#endif

// ---- the vocabulary ---------------------------------------------------------

static_assert(i2c_speed_hz(I2cSpeed::standard_100k) == 100'000);
static_assert(i2c_speed_hz(I2cSpeed::fast_400k) == 400'000);
static_assert(i2c_speed_hz(I2cSpeed::fast_plus_1m) == 1'000'000);
static_assert(i2c_clock_valid(I2cClock::hsi16));
static_assert(!i2c_clock_valid(static_cast<I2cClock>(3)));
static_assert(i2c_oa2_compared_bits(I2cOa2Mask::none) == 7);
static_assert(i2c_oa2_compared_bits(I2cOa2Mask::low_3) == 4);
static_assert(i2c_oa2_compared_bits(I2cOa2Mask::all) == 0);
static_assert(i2c_address_valid(0x7F, I2cAddressMode::seven_bit));
static_assert(!i2c_address_valid(0x80, I2cAddressMode::seven_bit));
static_assert(i2c_address_valid(0x3FF, I2cAddressMode::ten_bit));
static_assert(!i2c_address_valid(0x400, I2cAddressMode::ten_bit));

// ---- TIMINGR, both ways -----------------------------------------------------

// The register word and its inverse, on table 173's Sm 100 kHz column.
static_assert(i2c_timingr(I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 0x30420F13u);
static_assert(i2c_timing_of(0x30420F13u).presc == 0x3);
static_assert(i2c_timing_of(0x30420F13u).scll == 0x13);
static_assert(i2c_timing_of(0x30420F13u).sclh == 0xF);
static_assert(i2c_timing_of(0x30420F13u).sdadel == 2);
static_assert(i2c_timing_of(0x30420F13u).scldel == 4);

// EVERY CELL of tables 172, 173 and 174, priced with its own footnoted
// tSYNC budget. (The driver header carries the same set; repeated here
// because the fixture is what the family sweep runs.)
static_assert(i2c_scl_hz(8'000'000UL, I2cTiming{0x1, 0xC7, 0xC3, 2, 4}, 1000u) == 10'000);
static_assert(i2c_scl_hz(8'000'000UL, I2cTiming{0x1, 0x13, 0xF, 2, 4}, 1000u) == 100'000);
static_assert(i2c_scl_hz(8'000'000UL, I2cTiming{0x0, 0x9, 0x3, 1, 3}, 750u) == 400'000);
static_assert(i2c_scl_hz(8'000'000UL, I2cTiming{0x0, 0x6, 0x3, 0, 1}, 655u) == 500'000);
static_assert(i2c_scl_hz(16'000'000UL, I2cTiming{0x3, 0xC7, 0xC3, 2, 4}, 1000u) == 10'000);
static_assert(i2c_scl_hz(16'000'000UL, I2cTiming{0x3, 0x13, 0xF, 2, 4}, 1000u) == 100'000);
static_assert(i2c_scl_hz(16'000'000UL, I2cTiming{0x1, 0x9, 0x3, 2, 3}, 750u) == 400'000);
static_assert(i2c_scl_hz(16'000'000UL, I2cTiming{0x0, 0x4, 0x2, 0, 2}, 500u) == 1'000'000);
static_assert(i2c_scl_hz(48'000'000UL, I2cTiming{0xB, 0xC7, 0xC3, 2, 4}, 1000u) == 10'000);
static_assert(i2c_scl_hz(48'000'000UL, I2cTiming{0xB, 0x13, 0xF, 2, 4}, 1000u) == 100'000);
static_assert(i2c_scl_hz(48'000'000UL, I2cTiming{0x5, 0x9, 0x3, 3, 3}, 750u) == 400'000);
static_assert(i2c_scl_hz(48'000'000UL, I2cTiming{0x5, 0x3, 0x1, 0, 1}, 250u) == 1'000'000);

// The four field formulas, with SDADEL's missing +1 pinned on its own.
static_assert(i2c_scll_cycles(I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 80);
static_assert(i2c_sclh_cycles(I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 64);
static_assert(i2c_scldel_cycles(I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 20);
static_assert(i2c_sdadel_cycles(I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 8);
// 32.4.8's minimum stretch after every SCL falling edge.
static_assert(i2c_min_stretch_cycles(I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 29);

// ---- the chooser at the bench's own ladder: 64, 16 and 2 MHz ----------------

static_assert(i2c_timing_for(64'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(i2c_timing_for(64'000'000UL, I2cSpeed::fast_400k).has_value());
static_assert(i2c_timing_for(64'000'000UL, I2cSpeed::fast_plus_1m).has_value());
static_assert(i2c_timing_for(16'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(i2c_timing_for(16'000'000UL, I2cSpeed::fast_400k).has_value());
// 16 MHz CANNOT do Fm+: ES0548 2.10.1 asks 20 MHz for it.
static_assert(!i2c_timing_for(16'000'000UL, I2cSpeed::fast_plus_1m).has_value());
// 2 MHz CANNOT DO ANY OF THE THREE: the erratum's Sm floor is 4 MHz -
// which is why an I2C on a board that scales its core to 2 MHz has to
// take HSI16 as its kernel clock, and why the independent clock is not
// a luxury on this family.
static_assert(!i2c_timing_for(2'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(!i2c_timing_for(2'000'000UL, I2cSpeed::fast_400k).has_value());
static_assert(!i2c_timing_for(2'000'000UL, I2cSpeed::fast_plus_1m).has_value());

// The produced rate is inside the mode's band and NEVER above it.
static_assert(i2c_scl_hz(64'000'000UL, *i2c_timing_for(64'000'000UL,
                                                       I2cSpeed::standard_100k),
                         I2cSpeed::standard_100k) == 99'378);
static_assert(i2c_scl_hz(64'000'000UL, *i2c_timing_for(64'000'000UL, I2cSpeed::fast_400k),
                         I2cSpeed::fast_400k) == 400'000);
static_assert(i2c_scl_hz(64'000'000UL,
                         *i2c_timing_for(64'000'000UL, I2cSpeed::fast_plus_1m),
                         I2cSpeed::fast_plus_1m) == 1'000'000);
static_assert(i2c_scl_hz(16'000'000UL, *i2c_timing_for(16'000'000UL,
                                                       I2cSpeed::standard_100k),
                         I2cSpeed::standard_100k) == 100'000);
static_assert(i2c_scl_hz(16'000'000UL, *i2c_timing_for(16'000'000UL, I2cSpeed::fast_400k),
                         I2cSpeed::fast_400k) == 400'000);

// Every chosen value meets 32.4.5's two conditions at its own clock.
static_assert(i2c_setup_ok(64'000'000UL,
                           *i2c_timing_for(64'000'000UL, I2cSpeed::standard_100k),
                           i2c_bus_timing(I2cSpeed::standard_100k)));
static_assert(i2c_setup_ok(64'000'000UL,
                           *i2c_timing_for(64'000'000UL, I2cSpeed::fast_plus_1m),
                           i2c_bus_timing(I2cSpeed::fast_plus_1m)));
static_assert(i2c_hold_ok(64'000'000UL,
                          *i2c_timing_for(64'000'000UL, I2cSpeed::standard_100k),
                          I2cFilters{}, i2c_bus_timing(I2cSpeed::standard_100k)));
static_assert(i2c_hold_ok(16'000'000UL,
                          *i2c_timing_for(16'000'000UL, I2cSpeed::fast_400k),
                          I2cFilters{}, i2c_bus_timing(I2cSpeed::fast_400k)));

// 32.4.3's own conditions, which bind independently of the erratum.
static_assert(i2c_clock_requirements_met(64'000'000UL, I2cSpeed::fast_plus_1m,
                                         I2cFilters{}));
static_assert(!i2c_clock_requirements_met(64'000'000UL, I2cSpeed::fast_plus_1m,
                                          I2cFilters{true, 15}));
static_assert(i2c_datasheet_min_kernel_hz(I2cSpeed::fast_plus_1m, I2cFilters{}) ==
              18'000'000UL);
static_assert(i2c_datasheet_min_kernel_hz(I2cSpeed::fast_plus_1m,
                                          I2cFilters{false, 1}) == 16'000'000UL);

// ---- the time-out arithmetic (32.4.13, tables 177..179) ---------------------

static_assert(*i2c_timeout_code_for(16'000'000UL, 25'000u, false) == 0xC3);
static_assert(*i2c_timeout_code_for(48'000'000UL, 25'000u, false) == 0x249);
static_assert(*i2c_timeout_code_for(16'000'000UL, 50u, true) == 0xC7);
static_assert(i2c_timeout_us(16'000'000UL, 0xC3, false) == 25'088);
static_assert(i2c_timeout_us(16'000'000UL, 0xC7, true) == 50);
static_assert(!i2c_timeout_code_for(64'000'000UL, 1'000'000u, false).has_value());
static_assert(i2c_smbus_timeout_min_us == 25'000 && i2c_smbus_timeout_max_us == 35'000);
static_assert(i2c_smbus_host_address == 0x08 && i2c_smbus_device_default == 0x61);

// ---- the configurations -----------------------------------------------------

constexpr I2cConfig good{.timing = I2cTiming{1, 39, 15, 7, 13}};
static_assert(i2c_config_valid(good));
static_assert(i2c_cr1(good) == 0u);
constexpr I2cConfig filtered{.timing = good.timing, .filters = {false, 5}};
static_assert(i2c_cr1(filtered) == (I2C_CR1_ANFOFF | (5u << I2C_CR1_DNF_Pos)));
// 32.4.8: byte control and no-stretch are not compatible.
static_assert(!i2c_config_valid(I2cConfig{.timing = good.timing,
                                          .no_stretch = true, .byte_control = true}));
// 32.9.1: WUPEN needs DNF = 0 (a hardware interlock).
static_assert(!i2c_config_valid(I2cConfig{.timing = good.timing,
                                          .filters = {true, 1},
                                          .wake_from_stop = true}));
// 32.4.16: the wake needs stretching.
static_assert(!i2c_config_valid(I2cConfig{.timing = good.timing,
                                          .no_stretch = true,
                                          .wake_from_stop = true}));
static_assert(i2c_config_valid(I2cConfig{.timing = good.timing,
                                         .wake_from_stop = true}));

constexpr I2cAddressConfig addr7{.own = 0x42};
static_assert(i2c_address_config_valid(addr7));
static_assert(i2c_address_config_valid(I2cAddressConfig{
    .own = 0x155, .mode = I2cAddressMode::ten_bit}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x155}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x10, .second = 0x80}));

// ---- pads --------------------------------------------------------------------

constexpr I2cPins host_pins{
    .scl = {'B', 8, PinFunction::af6},
    .sda = {'B', 9, PinFunction::af6},
};
constexpr I2cPins client_pins{
    .scl = {'A', 11, PinFunction::af6},
    .sda = {'A', 12, PinFunction::af6},
};
static_assert(i2c_pins_valid(host_pins) && i2c_pins_valid(client_pins));
static_assert(!host_pins.has_alert());
static_assert(!i2c_pins_valid(I2cPins{.scl = {'B', 8, PinFunction::af6}}));
static_assert(!i2c_pins_valid(I2cPins{.scl = {'B', 8, PinFunction::af6},
                                      .sda = {'B', 8, PinFunction::af6}}));

// ---- the resource and the two tasks, instantiated ---------------------------

using One = I2c<1>;
using Two = I2c<2>;
static_assert(One::has_independent_clock && One::has_smbus && One::wakes_from_stop);
static_assert(One::exti_line == 23);
static_assert(One::has_ten_bit && One::has_fast_plus);

using Host = I2cHost<1, host_pins>;
using Peer = I2cClient<2, client_pins>;
static_assert(!Host::has_engines);
static_assert(std::is_trivially_copyable_v<Host::Request>);

// The engined host, on two channels of DMA1.
using HostTx = DmaTxEngine<1, 1>;
using HostRx = DmaRxEngine<1, 2>;
using DmaHost = I2cHost<1, host_pins, HostTx, HostRx>;
static_assert(DmaHost::has_engines);

#if defined(I2C3_BASE)
using Three = I2c<3>;
static_assert(!Three::has_independent_clock && !Three::has_smbus);
static_assert(Three::exti_line == 0xFF);
// I2C3's pads on the G0B1: PB3/PB4 at AF6 (DS13560 table 15).
constexpr I2cPins third_pins{
    .scl = {'B', 3, PinFunction::af6},
    .sda = {'B', 4, PinFunction::af6},
};
using ThirdHost = I2cHost<3, third_pins>;
#endif

void use() {
    SysClock clock;
    (void)One::regs();
    (void)Two::regs();
    (void)One::irq();
    (void)One::dma_rx_request();
    (void)One::tx_address();
    (void)One::rx_address();
    One::bus_clock(true);
    One::reset();
    (void)One::kernel_clock(I2cClock::hsi16);
    (void)One::kernel_clock();
    (void)One::kernel_hz(64'000'000UL);
    (void)One::enabled();
    One::enable();
    (void)One::disable();
    (void)One::cycle();
    (void)One::configure(good);
    (void)One::timing(good.timing);
    (void)One::timing();
    (void)One::timing_reg();
    (void)One::filters(I2cFilters{});
    (void)One::filters();
    (void)One::no_stretch(true);
    (void)One::no_stretch();
    One::byte_control(true);
    (void)One::byte_control();
    One::general_call(true);
    (void)One::general_call();
    (void)One::addresses(addr7);
    (void)One::oar1();
    (void)One::oar2();
    (void)One::own_address();
    (void)One::own_address10();
    (void)One::transfer(0x42, false, 4, true);
    (void)One::transfer(0x155, true, 4, true, false, I2cAddressMode::ten_bit, true);
    One::start();
    (void)One::starting();
    One::stop();
    (void)One::reload(255, true, false);
    (void)One::nbytes();
    (void)One::cr2();
    One::nack_next(true);
    (void)One::pec_byte(true);
    (void)One::pec();
    (void)One::data();
    One::data(0x5A);
    One::flush_tx();
    (void)One::flags();
    (void)One::flag(I2cFlag::busy);
    One::clear(I2cClear::all);
    One::clear_all();
    (void)One::busy();
    (void)One::host_reads();
    (void)One::matched_address();
    One::interrupt(I2cInterrupt::all, true);
    (void)One::interrupts();
    (void)One::pending();
    One::dma_transmit(true);
    One::dma_receive(true);
    (void)One::dma_transmit();
    (void)One::dma_receive();
    (void)One::timeouts(0xC3, false, true, 0x3E, true);
    (void)One::timeouts();
    (void)One::smbus_probe();
    (void)One::wake_from_stop(true);
    (void)One::wake_from_stop();
    One::instance_fast_plus(true);
    (void)One::instance_fast_plus();
    (void)One::pad_fast_plus('B', 8, true);
    (void)One::isr();
    One::release();

    (void)Host::init(clock, I2cClock::hsi16);
    Host::rebase(16'000'000UL);
    (void)Host::speed_ok(I2cSpeed::fast_400k);
    (void)Host::scl_hz(I2cSpeed::fast_400k);
    (void)Host::timing_of(I2cSpeed::fast_400k);
    (void)Host::kernel_hz();
    (void)Host::reference_hz();
    Host::fast_plus_drive(true);
    (void)Host::fast_plus_drive();
    (void)Host::idle();
    (void)Host::spurious_bus_errors();
    Host::Request r{};
    r.addr = 0x42;
    r.tx_len = 0;
    r.rx_len = 0;
    (void)Host::start(r);
    (void)Host::status();
    (void)Host::isr();
    (void)Host::dma_isr();
    (void)Host::unstick();
    (void)Host::recover();
    Host::release();

    (void)DmaHost::init(clock);
    (void)DmaHost::dma_isr();
    DmaHost::release();

    (void)Peer::init(clock, addr7, I2cSpeed::fast_400k);
    Peer::general_call(true);
    (void)Peer::byte_control(true);
    (void)Peer::addressed();
    (void)Peer::host_reads();
    (void)Peer::matched_address();
    Peer::answer_address();
    (void)Peer::data_ready();
    (void)Peer::take();
    (void)Peer::data_wanted();
    Peer::give(0x5A);
    Peer::flush();
    (void)Peer::answer_byte(true);
    (void)Peer::stop_seen();
    Peer::clear_stop();
    (void)Peer::host_nacked();
    Peer::clear_nack();
    (void)Peer::overrun();
    Peer::clear_overrun();
    (void)Peer::flags();
    Peer::clear(I2cClear::all);
    (void)Peer::wake_from_stop(true);
    (void)Peer::wake_from_stop();
    (void)Peer::isr();
    Peer::interrupt(I2cInterrupt::addr, true);
    Peer::release();

#if defined(I2C3_BASE)
    (void)Three::regs();
    (void)Three::kernel_clock(I2cClock::hsi16);   // no multiplexer: false
    (void)Three::timeouts(1, false, true, 1, true);   // no SMBus: false
    (void)Three::wake_from_stop(true);                // no wake: false
    (void)ThirdHost::init(clock);
    ThirdHost::release();
#endif
}
