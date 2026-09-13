// I2C family smoke TU: the resource over the whole of the chapter, the
// two bus tasks and the timing arithmetic, on every header of the pack.
// I2C1 and I2C2 exist on every F4; I2C3 is asked of the header (the F410
// has none). Every verb of every type is called once, and the facts the
// reserve derives are asserted against the header's own symbols.
#include <type_traits>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/i2c.hpp"

using namespace brio;

// ---- the reserve's presence facts ------------------------------------------------

static_assert(i2c_present(1) && i2c_present(2));
static_assert(!i2c_present(0) && !i2c_present(4));
static_assert(i2c_base(1) == I2C1_BASE);
static_assert(i2c_clock_mask(1) == RCC_APB1ENR_I2C1EN);
static_assert(i2c_event_irq(1) == I2C1_EV_IRQn && i2c_error_irq(1) == I2C1_ER_IRQn);
static_assert(i2c_clock_mask(2) == RCC_APB1ENR_I2C2EN);
static_assert(i2c_event_irq(2) == I2C2_EV_IRQn && i2c_error_irq(2) == I2C2_ER_IRQn);
// Every instance of this family is on APB1, and the reserve asks the
// enable bit's own register rather than saying so twice.
static_assert(!i2c_on_apb2(1) && !i2c_on_apb2(2) && !i2c_on_apb2(3));

#if defined(I2C3_BASE)
static_assert(i2c_present(3) && i2c_clock_mask(3) == RCC_APB1ENR_I2C3EN);
static_assert(i2c_event_irq(3) == I2C3_EV_IRQn && i2c_error_irq(3) == I2C3_ER_IRQn);
#else
// The F410 is the one part class of the pack with two I2C instances.
static_assert(!i2c_present(3));
static_assert(i2c_event_irq(3) == NonMaskableInt_IRQn);
#endif

// The noise filters: everywhere but the F405 class, whose RM0090 27.3.5
// gives the digital one to the F42x/F43x alone and whose header carries
// no I2C_FLTR_ANOFF.
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
static_assert(!i2c_has_filter() && !I2c<1>::has_filter);
#else
static_assert(i2c_has_filter() && I2c<1>::has_filter);
#endif

// FMPI2C1 is another block and has no driver here - the reserve publishes
// what the header knows and nothing more.
#if defined(FMPI2C1_BASE)
static_assert(fmpi2c_present() && fmpi2c_base() == FMPI2C1_BASE);
static_assert(fmpi2c_clock_mask() == RCC_APB1ENR_FMPI2C1EN);
static_assert(fmpi2c_event_irq() == FMPI2C1_EV_IRQn && fmpi2c_error_irq() == FMPI2C1_ER_IRQn);
#else
static_assert(!fmpi2c_present() && fmpi2c_clock_mask() == 0u);
static_assert(fmpi2c_event_irq() == NonMaskableInt_IRQn);
#endif

// ---- the DMA request mapping, per part class ---------------------------------------

#if defined(STM32F411xE)
// RM0383 table 27: the F411 alone gives I2C1_TX a third cell and I2C3_TX
// a second, and it shares the second I2C3_RX cell with the F446.
static_assert(i2c_dma_placements(1, true).known && i2c_dma_placements(1, true).count == 3);
static_assert(i2c_dma_placement_valid(1, true, 1, 1, 0));
static_assert(i2c_dma_placements(3, true).count == 2 && i2c_dma_placement_valid(3, true, 1, 5, 6));
static_assert(i2c_dma_placement_valid(3, false, 1, 1, 1));
#elif defined(STM32F446xx)
static_assert(i2c_dma_placements(1, true).count == 2 && !i2c_dma_placement_valid(1, true, 1, 1, 0));
static_assert(i2c_dma_placements(3, false).count == 2 && i2c_dma_placement_valid(3, false, 1, 1, 1));
static_assert(i2c_dma_placements(3, true).count == 1);
#elif defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || \
    defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F437xx) || \
    defined(STM32F429xx) || defined(STM32F439xx)
static_assert(i2c_dma_placements(3, false).known && i2c_dma_placements(3, false).count == 1);
static_assert(!i2c_dma_placement_valid(3, false, 1, 1, 1));
#else
// A part class whose manual was not read has no table, and an engine is
// refused there rather than run on a guessed channel.
static_assert(!i2c_dma_placements(1, true).known);
static_assert(!i2c_dma_placement_valid(1, true, 1, 6, 1));
#endif

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || \
    defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F437xx) || \
    defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F411xE)
// The cells every read class shares - all of them on DMA1, this block
// having no request on the other controller.
static_assert(i2c_dma_placement_valid(1, false, 1, 0, 1));
static_assert(i2c_dma_placement_valid(1, false, 1, 5, 1));
static_assert(i2c_dma_placement_valid(1, true, 1, 6, 1) && i2c_dma_placement_valid(1, true, 1, 7, 1));
static_assert(i2c_dma_placement_valid(2, false, 1, 2, 7) && i2c_dma_placement_valid(2, false, 1, 3, 7));
static_assert(i2c_dma_placement_valid(2, true, 1, 7, 7));
static_assert(!i2c_dma_placement_valid(1, true, 2, 6, 1));   // never the other controller
#if defined(I2C3_BASE)
static_assert(i2c_dma_placement_valid(3, false, 1, 2, 3) && i2c_dma_placement_valid(3, true, 1, 4, 3));
#endif
#endif

// ---- the clock ---------------------------------------------------------------------

using SysClock = Clock<ClockSource::hsi, 16'000'000>;
constexpr SysClock sys_clock;

// The arithmetic at the rate the HSI leaves on APB1, and at the two APB1
// ceilings of this family. Pinned here as well as in the header, because
// this TU is what compiles for the parts that have no board.
static_assert(i2c_timing_for(16'000'000UL, I2cSpeed::standard_100k)->ccr == 80u);
static_assert(i2c_timing_for(16'000'000UL, I2cSpeed::standard_100k)->freq_mhz == 16u);
static_assert(i2c_timing_for(16'000'000UL, I2cSpeed::standard_100k)->trise == 17u);
static_assert(i2c_timing_for(16'000'000UL, I2cSpeed::fast_400k)->ccr == (I2C_CCR_FS | 14u));
static_assert(i2c_scl_hz(16'000'000UL, *i2c_timing_for(16'000'000UL, I2cSpeed::fast_400k)) ==
              380'952UL);

// ---- the pads ---------------------------------------------------------------------

// I2C1 on PB6/PB7 (AF4), the pair every package of this family bonds.
constexpr I2cPins i2c1_pins{.scl = {'B', 6, PinFunction::af4}, .sda = {'B', 7, PinFunction::af4}};
static_assert(i2c_pins_valid(i2c1_pins));
// The alternate pad of SDA that the datasheets put on AF9, and an SMBus
// alert beside the two lines.
constexpr I2cPins i2c1_alt{.scl = {'B', 8, PinFunction::af4},
                           .sda = {'B', 9, PinFunction::af4},
                           .smba = {'B', 5, PinFunction::af4}};
static_assert(i2c_pins_valid(i2c1_alt));
constexpr I2cPins i2c2_pins{.scl = {'B', 10, PinFunction::af4}, .sda = {'B', 3, PinFunction::af9}};
static_assert(i2c_pins_valid(i2c2_pins));
// A bus needs both lines, and two signals may not share a pad.
static_assert(!i2c_pins_valid(I2cPins{.scl = {'B', 6, PinFunction::af4}}));
static_assert(!i2c_pins_valid(I2cPins{.sda = {'B', 7, PinFunction::af4}}));
static_assert(!i2c_pins_valid(I2cPins{.scl = {'B', 6, PinFunction::af4},
                                      .sda = {'B', 7, PinFunction::af4},
                                      .smba = {'B', 6, PinFunction::af4}}));

// ---- the resource, verb by verb ------------------------------------------------------

template <uint8_t n>
void resource_smoke() {
    using S = I2c<n>;
    static_assert(S::number == n);
    static_assert(S::event_irq == i2c_event_irq(n) && S::error_irq == i2c_error_irq(n));
    static_assert(!S::on_apb2);

    S::bus_clock(true);
    (void)S::bus_clock();
    S::reset();

    const auto t = i2c_timing_for(16'000'000UL, I2cSpeed::fast_400k, I2cDuty::ratio_16_9, 250);
    S::timing(t.value_or(I2cTiming{}));
    (void)S::timing();
    (void)S::filter(I2cFilter{.analog = false, .digital = 4});
    (void)S::filter(I2cFilter{});
    (void)S::filter();

    (void)S::addresses(I2cAddressConfig{.own = 0x41});
    (void)S::addresses(I2cAddressConfig{.own = 0x41, .second = 0x42, .general_call = true});
    (void)S::addresses(I2cAddressConfig{.own = 0x123, .ten_bit = true});
    (void)S::addresses(I2cAddressConfig{.own = 0x800});   // refused

    S::enable();
    (void)S::enabled();
    S::disable();
    S::software_reset();

    S::start();
    (void)S::starting();
    S::stop();
    (void)S::stopping();
    S::ack(true);
    S::pos(true);
    S::pos(false);
    S::general_call(false);
    S::no_stretch(true);
    S::no_stretch(false);

    S::smbus(I2cSmbusConfig{.enabled = true, .host = true, .arp = true});
    (void)S::smbus();
    S::smbus(I2cSmbusConfig{});
    S::alert(true);
    S::alert(false);
    S::pec(true);
    (void)S::pec();
    S::pec_transfer();
    (void)S::pec_pending();
    (void)S::pec_value();
    S::pec(false);

    S::dma(true, true);
    S::dma(false);
    (void)S::data_address();

    S::data(0x5Au);
    (void)S::data();
    (void)S::status1();
    (void)S::status2();
    (void)S::flag(I2cFlag::byte_finished);
    (void)S::busy();
    (void)S::host();
    (void)S::transmitting();
    (void)S::second_address_matched();
    (void)S::general_call_matched();
    (void)S::smbus_host_matched();
    (void)S::smbus_default_matched();
    (void)S::clear_addr();
    S::clear_stopf();
    S::clear_errors(I2cFlag::errors);

    S::event_interrupt(true);
    S::buffer_interrupt(true);
    S::error_interrupt(true);
    S::bus_clock(false);
}

// ---- the host task -----------------------------------------------------------------

void host_smoke() {
    using Bus = I2cHost<1, i2c1_pins>;
    static_assert(!Bus::has_engines);
    static_assert(std::is_same_v<Bus::Resource, I2c<1>>);
    static_assert(Bus::pin_sel.scl.pin == 6);

    (void)Bus::init(sys_clock);
    (void)Bus::init(sys_clock, I2cHostConfig{.duty = I2cDuty::ratio_16_9,
                                             .standard_rise_ns = 400,
                                             .fast_rise_ns = 120,
                                             .filter = {.analog = true, .digital = 1}});
    Bus::rebase(16'000'000UL);
    (void)Bus::speed_ok(I2cSpeed::fast_400k);
    (void)Bus::timing_of(I2cSpeed::standard_100k);
    (void)Bus::scl_hz(I2cSpeed::standard_100k);
    (void)Bus::reference_hz();
    (void)Bus::digital_filter_max(I2cSpeed::fast_400k);
    (void)Bus::repeated_start_setup_at_risk(I2cSpeed::standard_100k);
    (void)Bus::spurious_bus_errors();
    Bus::clear_spurious_bus_errors();

    static uint8_t out[4] = {1, 2, 3, 4};
    static uint8_t in[4] = {};
    Bus::Request r{};
    r.addr = 0x41;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out));
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(in));
    r.rx_len = 2;
    r.speed = I2cSpeed::fast_400k;
    (void)Bus::start(r);
    (void)Bus::isr();
    (void)Bus::error_isr();
    (void)Bus::dma_isr();
    (void)Bus::status();
    (void)Bus::unstick();
    (void)Bus::recover();
    Bus::claim_smba_pad(true);
    Bus::release();

    // The SMBus alert pad, on a bus that names one.
    using Alerted = I2cHost<1, i2c1_alt>;
    (void)Alerted::init(sys_clock);
    Alerted::claim_smba_pad(true);
    Alerted::claim_smba_pad(false);
    Alerted::release();
}

/// The engine slots are behind the reserve's `known`: on a part class
/// whose request table was not read the two slots stay empty, because a
/// stream on a guessed channel is exactly what the static_assert refuses.
template <bool known = i2c_dma_placements(1, true).known>
void engined_smoke() {
    using Tx = std::conditional_t<known, DmaTxEngine<1, 6, 1>, NoDmaEngine>;
    using Rx = std::conditional_t<known, DmaRxEngine<1, 0, 1>, NoDmaEngine>;
    using EnginedBus = I2cHost<1, i2c1_pins, Tx, Rx>;
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

// ---- the client task ----------------------------------------------------------------

void client_smoke() {
    using Peer = I2cClient<1, i2c1_pins>;
    static_assert(std::is_same_v<Peer::Resource, I2c<1>>);

    (void)Peer::init(sys_clock, I2cAddressConfig{.own = 0x2A});
    (void)Peer::init(sys_clock, I2cAddressConfig{.own = 0x2A, .second = 0x2B}, true);
    (void)Peer::addressed();
    (void)Peer::answer_address();
    (void)Peer::second_address_matched();
    (void)Peer::general_call_matched();
    (void)Peer::data_ready();
    (void)Peer::take();
    (void)Peer::data_wanted();
    Peer::give(0x33u);
    Peer::acknowledge(true);
    (void)Peer::stop_seen();
    Peer::clear_stop();
    (void)Peer::host_nacked();
    Peer::clear_nack();
    (void)Peer::overrun();
    Peer::clear_overrun();
    (void)Peer::flags();
    (void)Peer::service();
    (void)Peer::error_service();
    (void)Peer::host_reads();
    Peer::release();
}

int main() {
    resource_smoke<1>();
    resource_smoke<2>();
#if defined(I2C3_BASE)
    resource_smoke<3>();
#endif
    host_smoke();
    engined_smoke();
    client_smoke();
    return 0;
}
