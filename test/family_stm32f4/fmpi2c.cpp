// FMPI2C family smoke TU: the Fast-mode Plus I2C's whole vocabulary and
// arithmetic on every header of the pack, and its resource and two tasks
// wherever the header declares the block - the F410, F412, F413/F423 and
// F446, and nowhere else. The arithmetic half of stm32f4/fmpi2c.hpp names
// no register and must therefore compile on all twenty-three; the
// register half is behind FMPI2C1_BASE, as dac.hpp's is behind DAC_BASE,
// and this TU checks that the split holds by exercising each half where it
// belongs.
#include <type_traits>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/fmpi2c.hpp"
#include "stm32f4/syscfg.hpp"

using namespace brio;

// ---- the reserve's facts, against the header's own symbols -------------------------

#if defined(FMPI2C1_BASE)
static_assert(fmpi2c_present() && fmpi2c_base() == FMPI2C1_BASE);
static_assert(fmpi2c_clock_mask() == RCC_APB1ENR_FMPI2C1EN);
static_assert(fmpi2c_event_irq() == FMPI2C1_EV_IRQn && fmpi2c_error_irq() == FMPI2C1_ER_IRQn);
// The two vectors are not one another's, and neither is any I2C's.
static_assert(fmpi2c_event_irq() != fmpi2c_error_irq());
static_assert(fmpi2c_event_irq() != i2c_event_irq(1));
// The Fm+ pad drive lives in SYSCFG_CFGR, and the register exists exactly
// where the peripheral does.
static_assert(Syscfg::has_fast_mode_plus());
#else
static_assert(!fmpi2c_present() && fmpi2c_clock_mask() == 0u);
static_assert(fmpi2c_event_irq() == NonMaskableInt_IRQn);
static_assert(fmpi2c_error_irq() == NonMaskableInt_IRQn);
static_assert(!Syscfg::has_fast_mode_plus());
#endif

// TABLE 127's DASH: no part of this pack gives this block a wake from
// Stop, and the header says so in the only way it can - CR1's bit 18 has
// no name anywhere.
static_assert(!fmpi2c_has_wakeup());

// The SMBus row is a MANUAL's claim and only the F446's was read.
#if defined(STM32F446xx)
static_assert(fmpi2c_smbus_claimed());
#else
static_assert(!fmpi2c_smbus_claimed());
#endif

// ---- the DMA request mapping, per part class ---------------------------------------

#if defined(STM32F446xx)
// RM0390 table 28: one cell per direction, both on DMA1 channel 2.
static_assert(fmpi2c_dma_placements(true).known && fmpi2c_dma_placements(true).count == 1);
static_assert(fmpi2c_dma_placement_valid(true, 1, 5, 2));
static_assert(fmpi2c_dma_placement_valid(false, 1, 2, 2));
static_assert(!fmpi2c_dma_placement_valid(true, 1, 2, 2));   // the receive cell, not this one
static_assert(!fmpi2c_dma_placement_valid(false, 2, 2, 2));  // never the other controller
#else
// Every other part class - the ones with the block and no manual on this
// desk, and the ones without the block at all - has no table, and an
// engine is refused there rather than run on a guessed stream.
static_assert(!fmpi2c_dma_placements(true).known);
static_assert(!fmpi2c_dma_placement_valid(true, 1, 5, 2));
#endif

// ---- the arithmetic, on every header ------------------------------------------------

// The three speeds and their rates.
static_assert(fmpi2c_speed_count == 3);
static_assert(fmpi2c_speed_hz(FmpI2cSpeed::standard_100k) == 100'000UL);
static_assert(fmpi2c_speed_hz(FmpI2cSpeed::fast_400k) == 400'000UL);
static_assert(fmpi2c_speed_hz(FmpI2cSpeed::fast_plus_1m) == 1'000'000UL);
static_assert(fmpi2c_clock_valid(FmpI2cClock::hsi) &&
              !fmpi2c_clock_valid(static_cast<FmpI2cClock>(3)));

// The chapter's own tables, at the two kernel clocks it prints and at the
// three a 180 MHz part can put under the block. (The header carries the
// full set; these are the cells this TU is the only check for on a part
// with no board.)
static_assert(fmpi2c_scl_hz(8'000'000UL, FmpI2cTiming{0x1, 0x13, 0xF, 2, 4}, 1000u) == 100'000);
static_assert(fmpi2c_scl_hz(16'000'000UL, FmpI2cTiming{0x0, 0x4, 0x2, 0, 2}, 500u) == 1'000'000);
static_assert(fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::fast_plus_1m).has_value());
static_assert(fmpi2c_timing_for(180'000'000UL, FmpI2cSpeed::fast_plus_1m).has_value());
static_assert(!fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::fast_plus_1m).has_value());
// At 16 MHz - the HSI, the one kernel rate every part of the family can
// reach whatever its core is doing - two of the three speeds are legal and
// the third is refused by ES0298 2.12.2's floor.
static_assert(fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::standard_100k).has_value());
static_assert(fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::fast_400k).has_value());
static_assert(fmpi2c_scl_hz(16'000'000UL,
                            *fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::standard_100k),
                            FmpI2cSpeed::standard_100k) == 100'000);
static_assert(fmpi2c_scl_hz(16'000'000UL,
                            *fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::fast_400k),
                            FmpI2cSpeed::fast_400k) == 400'000);
// The chosen value meets 23.4.5's two bounds and the chapter's minimum
// stretch is what its two delays make it.
static_assert(fmpi2c_setup_ok(16'000'000UL,
                              *fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::standard_100k),
                              fmpi2c_bus_timing(FmpI2cSpeed::standard_100k)));
static_assert(fmpi2c_hold_ok(16'000'000UL,
                             *fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::standard_100k),
                             FmpI2cFilters{},
                             fmpi2c_bus_timing(FmpI2cSpeed::standard_100k)));
static_assert(fmpi2c_hold_upper_ok(16'000'000UL,
                                   *fmpi2c_timing_for(16'000'000UL,
                                                      FmpI2cSpeed::standard_100k),
                                   FmpI2cFilters{},
                                   fmpi2c_bus_timing(FmpI2cSpeed::standard_100k),
                                   FmpI2cSpeed::standard_100k));
static_assert(fmpi2c_min_stretch_cycles(FmpI2cTiming{0x3, 0x13, 0xF, 2, 4}) == 29);
// A bus with measured edges, which is what the argument is for: a fast
// wire buys a shorter setup delay than the standard's worst case.
static_assert(fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::standard_100k, FmpI2cFilters{},
                                FmpI2cBusTiming{100, 100, 250, 1000})
                  ->scldel <=
              fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::standard_100k)->scldel);

// ---- the pads -----------------------------------------------------------------------

// DS10693 table 11: FMPI2C1_SCL on PC6, PD12, PD14 and PF14, SDA on PC7,
// PD13, PD15 and PF15, SMBA on PD11 and PF13 - AF4 on every one of them.
// The pair a 64-pin package bonds is PC6/PC7.
constexpr FmpI2cPins fmp_pins{.scl = {'C', 6, PinFunction::af4},
                              .sda = {'C', 7, PinFunction::af4}};
static_assert(fmpi2c_pins_valid(fmp_pins));
static_assert(!fmp_pins.has_alert());
constexpr FmpI2cPins fmp_alt{.scl = {'B', 10, PinFunction::af4},
                             .sda = {'B', 11, PinFunction::af4},
                             .smba = {'B', 12, PinFunction::af4}};
static_assert(fmpi2c_pins_valid(fmp_alt) && fmp_alt.has_alert());
// A bus needs both lines, and two signals may not share a pad.
static_assert(!fmpi2c_pins_valid(FmpI2cPins{.scl = {'C', 6, PinFunction::af4}}));
static_assert(!fmpi2c_pins_valid(FmpI2cPins{.sda = {'C', 7, PinFunction::af4}}));
static_assert(!fmpi2c_pins_valid(FmpI2cPins{.scl = {'C', 6, PinFunction::af4},
                                            .sda = {'C', 7, PinFunction::af4},
                                            .smba = {'C', 6, PinFunction::af4}}));

// ---- the clock ------------------------------------------------------------------------

using SysClock = Clock<ClockSource::hsi, 16'000'000>;
constexpr SysClock sys_clock;

#if defined(FMPI2C1_BASE)

// ---- the resource, verb by verb ---------------------------------------------------------

void resource_smoke() {
    using S = FmpI2c<1>;
    static_assert(S::number == 1);
    static_assert(S::event_irq == fmpi2c_event_irq() && S::error_irq == fmpi2c_error_irq());
    static_assert(S::has_ten_bit && S::has_fast_plus);
    static_assert(!S::wakes_from_stop);
    static_assert(S::smbus_claimed == fmpi2c_smbus_claimed());

    S::bus_clock(true);
    (void)S::bus_clock();
    S::reset();

    (void)S::kernel_clock(FmpI2cClock::hsi);
    (void)S::kernel_clock(FmpI2cClock::sysclk);
    (void)S::kernel_clock(FmpI2cClock::pclk);
    (void)S::kernel_clock(static_cast<FmpI2cClock>(3));   // refused
    (void)S::kernel_clock();
    (void)S::kernel_hz(16'000'000UL, 16'000'000UL);

    const auto t = fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::fast_400k);
    (void)S::timing(t.value_or(FmpI2cTiming{}));
    (void)S::timing();
    (void)S::timing_reg();
    (void)S::filters(FmpI2cFilters{.analog = false, .digital = 4});
    (void)S::filters(FmpI2cFilters{.digital = 16});   // refused
    (void)S::filters(FmpI2cFilters{});
    (void)S::filters();
    (void)S::no_stretch(true);
    (void)S::no_stretch(false);
    (void)S::no_stretch();
    S::byte_control(true);
    (void)S::byte_control();
    S::byte_control(false);
    S::general_call(true);
    (void)S::general_call();
    S::general_call(false);

    FmpI2cConfig cfg{};
    cfg.timing = t.value_or(FmpI2cTiming{});
    cfg.smbus_host = true;
    cfg.smbus_device = true;
    cfg.smbus_alert = true;
    cfg.pec = true;
    (void)S::configure(cfg);
    cfg.no_stretch = true;
    cfg.byte_control = true;
    (void)S::configure(cfg);   // refused: 23.4.8's incompatible pair

    (void)S::addresses(FmpI2cAddressConfig{.own = 0x41});
    (void)S::addresses(FmpI2cAddressConfig{.own = 0x41,
                                           .second = 0x42,
                                           .second_mask = FmpI2cOa2Mask::low_2,
                                           .second_enable = true});
    (void)S::addresses(FmpI2cAddressConfig{.own = 0x123, .mode = FmpI2cAddressMode::ten_bit});
    (void)S::addresses(FmpI2cAddressConfig{.own = 0x800});   // refused
    (void)S::oar1();
    (void)S::oar2();
    (void)S::own_address();
    (void)S::own_address10();

    S::enable();
    (void)S::enabled();
    (void)S::disable();
    (void)S::cycle();
    (void)S::disable();

    (void)S::transfer(0x41, false, 2, true);
    (void)S::transfer(0x123, true, 1, false, false, FmpI2cAddressMode::ten_bit, true);
    (void)S::transfer(0x41, false, 1, true, true);   // refused: RELOAD with AUTOEND
    (void)S::transfer_now(0x41, true, 4, true);
    (void)S::cr2_word(0x41, false, 1, true, false, FmpI2cAddressMode::seven_bit, false);
    S::start();
    (void)S::starting();
    S::stop();
    (void)S::reload(1, true, false);
    (void)S::reload(0, false, false);   // refused: a zero NBYTES clears nothing
    (void)S::nbytes();
    (void)S::cr2();
    S::nack_next(true);
    S::nack_next(false);
    S::pec_byte(true);
    S::pec_byte(false);
    (void)S::pec();
    (void)S::ten_bit_nack_stuck();

    S::data(0x5Au);
    (void)S::data();
    S::flush_tx();
    (void)S::tx_address();
    (void)S::rx_address();

    (void)S::flags();
    (void)S::flag(FmpI2cFlag::busy);
    S::clear(FmpI2cClear::nack);
    S::clear_all();
    (void)S::busy();
    (void)S::host_reads();
    (void)S::matched_address();

    S::interrupt(FmpI2cInterrupt::all, true);
    (void)S::interrupts();
    (void)S::pending();
    (void)S::pending_event();
    (void)S::pending_error();
    S::interrupt(FmpI2cInterrupt::all, false);

    S::dma_transmit(true);
    S::dma_receive(true);
    (void)S::dma_transmit();
    (void)S::dma_receive();
    S::dma_transmit(false);
    S::dma_receive(false);

    (void)S::timeouts(0x61, false, true, 0x1F, true);
    (void)S::timeouts(4096, false, false, 0, false);   // refused: past twelve bits
    (void)S::timeouts();
    (void)S::smbus_probe();

    // Table 127's dash, as a verb that refuses.
    (void)S::wake_from_stop(true);
    (void)S::wake_from_stop();

    S::fast_plus_drive(true, true);
    (void)S::fast_plus_scl();
    (void)S::fast_plus_sda();
    S::fast_plus_drive(false, false);

    (void)S::isr();
    (void)S::error_isr();
    S::release();
}

// ---- the host task -----------------------------------------------------------------------

void host_smoke() {
    using Bus = FmpI2cHost<1, fmp_pins>;
    static_assert(!Bus::has_engines);
    static_assert(std::is_same_v<Bus::Resource, FmpI2c<1>>);
    static_assert(Bus::pin_sel.scl.pin == 6);
    // THE REQUEST IS THE ARBITER'S, field for field - the whole point of
    // the task layer (docs/design/i2c-bus.md).
    static_assert(std::is_trivially_copyable_v<Bus::Request>);
    static_assert(std::is_same_v<decltype(Bus::Request::addr), uint8_t>);
    static_assert(std::is_same_v<decltype(Bus::Request::reply), ReplyTo<I2cDone>>);

    (void)Bus::init(sys_clock);
    (void)Bus::init(sys_clock, FmpI2cHostConfig{.kernel = FmpI2cClock::hsi,
                                                .filters = {.analog = true, .digital = 1},
                                                .internal_pull_up = true,
                                                .bus = FmpI2cBusTiming{200, 200, 250, 1000}});
    Bus::rebase(16'000'000UL);
    (void)Bus::speed_ok(FmpI2cSpeed::fast_plus_1m);
    (void)Bus::timing_of(FmpI2cSpeed::standard_100k);
    (void)Bus::scl_hz(FmpI2cSpeed::standard_100k);
    (void)Bus::kernel_hz();
    (void)Bus::reference_hz();
    (void)Bus::transmit_stall_risk();
    (void)Bus::spurious_bus_errors();
    (void)Bus::idle();
    Bus::fast_plus_drive(true);
    (void)Bus::fast_plus_drive();
    Bus::claim_smba_pad(true);   // no alert pad named: a no-op

    static uint8_t out[4] = {1, 2, 3, 4};
    static uint8_t in[4] = {};
    Bus::Request r{};
    r.addr = 0x41;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out));
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(in));
    r.rx_len = 2;
    r.speed = FmpI2cSpeed::fast_400k;
    (void)Bus::start(r);
    (void)Bus::isr();
    (void)Bus::error_isr();
    (void)Bus::dma_isr();
    (void)Bus::status();
    (void)Bus::unstick();
    (void)Bus::recover();
    Bus::release();

    // The SMBus alert pad, on a bus that names one.
    using Alerted = FmpI2cHost<1, fmp_alt>;
    (void)Alerted::init(sys_clock);
    Alerted::claim_smba_pad(true);
    Alerted::claim_smba_pad(false);
    Alerted::release();
}

/// The engine slots are behind the reserve's `known`: on a part class whose
/// request table was not read the two slots stay empty, because a stream on
/// a guessed channel is exactly what the static_assert refuses.
template <bool known = fmpi2c_dma_placements(true).known>
void engined_smoke() {
    using Tx = std::conditional_t<known, DmaTxEngine<1, 5, 2>, NoDmaEngine>;
    using Rx = std::conditional_t<known, DmaRxEngine<1, 2, 2>, NoDmaEngine>;
    using EnginedBus = FmpI2cHost<1, fmp_pins, Tx, Rx>;
    static_assert(EnginedBus::has_engines == known);
    (void)EnginedBus::init(sys_clock);
    static uint8_t out[2] = {0, 1};
    static uint8_t in[8] = {};
    typename EnginedBus::Request r{};
    r.addr = 0x41;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out));
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(in));
    r.rx_len = 8;
    (void)EnginedBus::start(r);
    (void)EnginedBus::dma_isr();
    (void)EnginedBus::isr();
    (void)EnginedBus::error_isr();
    (void)EnginedBus::status();
    (void)EnginedBus::recover();
    EnginedBus::release();
}

// ---- the client task ---------------------------------------------------------------------

void client_smoke() {
    using Peer = FmpI2cClient<1, fmp_pins>;
    static_assert(std::is_same_v<Peer::Resource, FmpI2c<1>>);

    (void)Peer::init(sys_clock, FmpI2cAddressConfig{.own = 0x2A});
    (void)Peer::init(sys_clock,
                     FmpI2cAddressConfig{.own = 0x2A,
                                         .second = 0x2B,
                                         .second_mask = FmpI2cOa2Mask::all,
                                         .second_enable = true},
                     FmpI2cSpeed::fast_400k, FmpI2cHostConfig{.kernel = FmpI2cClock::hsi}, true);
    (void)Peer::kernel_hz();
    Peer::general_call(true);
    (void)Peer::byte_control(true);
    (void)Peer::addressed();
    (void)Peer::host_reads();
    (void)Peer::matched_address();
    Peer::answer_address();
    (void)Peer::data_ready();
    (void)Peer::take();
    (void)Peer::data_wanted();
    Peer::give(0x33u);
    Peer::flush();
    (void)Peer::answer_byte(true);
    (void)Peer::stop_seen();
    Peer::clear_stop();
    (void)Peer::host_nacked();
    Peer::clear_nack();
    (void)Peer::overrun();
    Peer::clear_overrun();
    (void)Peer::flags();
    Peer::clear(FmpI2cClear::all);
    Peer::interrupt(FmpI2cInterrupt::addr, true);
    (void)Peer::service();
    (void)Peer::error_service();
    (void)Peer::host_reads_last();
    Peer::release();
}

#endif // FMPI2C1_BASE

int main() {
#if defined(FMPI2C1_BASE)
    resource_smoke();
    host_smoke();
    engined_smoke();
    client_smoke();
#endif
    return 0;
}
