// mcu: ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// I2C family smoke TU: the resource's verbs over the whole of RM
// chapter 19, the clock arithmetic in its three registers, the columns
// table 10-34 gives I2C1 and the single one I2C2 has, the host engine
// with its Request and its DMA slots, the client with its polled
// surface, and the arbiter over the whole of it.
//
// THE CH32V203F6 IS NOT IN THE LIST: that package has NO I2C at all
// (datasheet table 2-1 - I2C1 lives on PB6/PB7 and neither pad is
// bonded there), so the TU that names an instance cannot compile for
// it, and neg/i2c_absent.cpp is that refusal.
//
// HOW MANY INSTANCES a part has is device::i2c_count and nothing else:
// the parts below the CH32V203C8 carry I2C1 alone. So this TU names
// I2C2 only through `second_i2c`, which is the instance where there is
// one and I2C1 where there is not - and a neg TU proves that I2c<2> is
// refused on a part with one.
//
// WHICH COLUMN a program may name is two questions with two answers:
// the instance must HAVE the column (I2C1 has two, I2C2 has none - no
// remap field exists for it), and the package must BOND both pads. The
// default column of each instance is what this TU instantiates,
// because that is the one every part in the list can answer for.
#include <variant>

#include "ch32v203/clock.hpp"
#include "ch32v203/dma.hpp"
#include "ch32v203/i2c.hpp"
#include "ch32v203/platform.hpp"
#include "kernel/tenuto.hpp"
#include "util/i2c_bus.hpp"

using namespace brio;

using P = Ch32v203Platform<>;
/// 96 MHz of HCLK is 48 MHz of PB1: the top of the tree (144 MHz, PB1
/// at 72) has NO legal FREQ for this peripheral, which is the chapter's
/// own ceiling and not a choice of this file.
using SysClock = Clock<ClockSource::pll, 96'000'000>;

/// The second instance where the part has one, and the first where it
/// has not: a TU that must compile on eight parts names no literal 2.
inline constexpr uint8_t second_i2c = device::i2c_count >= 2u ? 2u : 1u;

using One = I2c<1>;
using Two = I2c<second_i2c>;

// ---- the family's own facts ------------------------------------------------

static_assert(i2c_base_for(1) == pb1_base + 0x5400 && i2c_base_for(2) == pb1_base + 0x5800);
static_assert(i2c_base_for(3) == 0, "this family addresses two I2C instances");
static_assert(i2c_bus_for(1) == Bus::pb1 && i2c_bus_for(2) == Bus::pb1,
              "both instances hang on PB1, which is the clock FREQ states");
static_assert(i2c_gate_for(1) == rcc_pb1_i2c1 && i2c_gate_for(2) == rcc_pb1_i2c2);
static_assert(i2c_event_irq_for(1) == Irq::i2c1_ev && i2c_error_irq_for(1) == Irq::i2c1_er);
static_assert(i2c_event_irq_for(2) == Irq::i2c2_ev && i2c_error_irq_for(2) == Irq::i2c2_er);
static_assert(i2c_present(1) == (device::i2c_count >= 1u));
static_assert(i2c_present(2) == (device::i2c_count >= 2u));
static_assert(!i2c_present(0) && !i2c_present(3));

static_assert(One::number == 1 && One::bus == Bus::pb1);
static_assert(One::dma_tx_channel == 6 && One::dma_rx_channel == 7,
              "table 11-5: I2C1 transmits on channel 6 and receives on 7");
static_assert(device::i2c_count < 2u ||
                  (I2c<second_i2c>::dma_tx_channel == 4 && I2c<second_i2c>::dma_rx_channel == 5),
              "table 11-5: I2C2 transmits on channel 4 and receives on 5");

// The two columns and their pads (table 10-34 and the datasheet's pin
// table), and the rule that judges them on THIS part.
static_assert(i2c_column_count(1) == 2u && i2c_column_count(2) == 1u);
static_assert(i2c_pins_for(1, 0).scl == Pad{'B', 6} && i2c_pins_for(1, 0).sda == Pad{'B', 7});
static_assert(i2c_pins_for(1, 1).scl == Pad{'B', 8} && i2c_pins_for(1, 1).sda == Pad{'B', 9});
static_assert(i2c_pins_for(2, 0).scl == Pad{'B', 10} && i2c_pins_for(2, 0).sda == Pad{'B', 11});
static_assert(i2c_default_pins<1>.remap == 0u && i2c_default_pins<2>.remap == 0u);
static_assert(i2c_pins_valid(1, i2c_default_pins<1>),
              "every part in this TU's list bonds PB6 and PB7");
static_assert(device::i2c_count < 2u || i2c_pins_valid(2, i2c_default_pins<2>));
// The refusals i2c_pins_valid() owes, on every part: one line missing,
// two signals on one pad, a column the instance has not got.
static_assert(!i2c_pins_valid(1, I2cPins{.scl = {'B', 6}}));
static_assert(!i2c_pins_valid(1, I2cPins{.scl = {'B', 6}, .sda = {'B', 6}}));
static_assert(!i2c_pins_valid(2, I2cPins{.scl = {'B', 10}, .sda = {'B', 11}, .remap = 1}));

// ---- the arithmetic, in the three registers --------------------------------

static_assert(i2c_speed_hz(I2cSpeed::standard_100k) == 100'000UL);
static_assert(i2c_speed_hz(I2cSpeed::fast_400k) == 400'000UL);
static_assert(i2c_default_rise_ns(I2cSpeed::standard_100k) == 1000u &&
              i2c_default_rise_ns(I2cSpeed::fast_400k) == 300u);
static_assert(i2c_bus_hz_at(96'000'000UL) == 48'000'000UL,
              "HCLK 96 MHz is PB1 48 MHz: the prescaler is the clock task's");
static_assert(i2c_bus_hz_at(48'000'000UL) == 48'000'000UL);
static_assert(i2c_bus_hz(SysClock{}) == SysClock::pclk1_hz);

constexpr auto sm = i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k);
constexpr auto fm = i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k);
constexpr auto fm169 = i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k, I2cDuty::ratio_16_9);
static_assert(sm && fm && fm169);
static_assert(sm->freq_mhz == 48u && sm->ckcfgr == 240u && sm->trise == 49u);
static_assert(fm->ckcfgr == (i2c_fs | 40u) && fm->trise == 15u);
static_assert(fm169->ckcfgr == (i2c_fs | i2c_duty | 5u));
static_assert(i2c_scl_hz(48'000'000UL, *sm) == 100'000UL);
static_assert(i2c_scl_hz(48'000'000UL, *fm) == 400'000UL);
static_assert(i2c_scl_hz(48'000'000UL, I2cTiming{}) == 0u, "a zero CCR is no rate at all");
// The wire as an argument: a microsecond of rise in fast mode.
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k, I2cDuty::ratio_2, 1000)->trise ==
              49u);
// 19.12.2's window, which is also what refuses PB1 at the stratum's own
// ceiling of 72 MHz.
static_assert(!i2c_timing_for(3'999'999UL, I2cSpeed::standard_100k).has_value());
static_assert(i2c_timing_for(4'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(i2c_timing_for(60'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(!i2c_timing_for(72'000'000UL, I2cSpeed::standard_100k).has_value());
// A rise time RTR's six bits cannot hold is no timing either.
static_assert(!i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k, I2cDuty::ratio_2, 2000)
                   .has_value());

static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x2C}));
static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x2C, .second = 0x39}));
static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x3FF, .ten_bit = true}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x80}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x400, .ten_bit = true}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x10, .second = 0x90}));

// The status codes the engine speaks are util/i2c_bus.hpp's, and the
// one code of this stratum's own sits above them.
static_assert(i2c_dma_fault > i2c_bus_error && i2c_dma_fault != i2c_timeout);

// ---- the resource's verbs --------------------------------------------------

void resource() {
    One::bus_clock(true);
    One::reset();
    (void)One::remap(0);
    (void)One::remap(1);
    One::timing(*sm);
    (void)One::addresses({.own = 0x2C, .second = 0x39, .general_call = true});
    (void)One::addresses({.own = 0x123, .ten_bit = true});
    One::enable();
    (void)One::enabled();
    One::software_reset();

    One::start();
    (void)One::starting();
    One::stop();
    (void)One::stopping();
    One::ack(true);
    One::pos(true);
    One::general_call(false);
    One::no_stretch(false);
    One::pec(true);
    One::pec_next(true);
    One::smbus(true, true);
    One::arp(true);
    One::alert(false);
    One::dma(true, true);
    One::dma(false);

    One::data(0x5A);
    (void)One::data();
    (void)One::status1();
    (void)One::status2();
    (void)One::flag(i2c_sb | i2c_btf | i2c_add10 | i2c_rxne | i2c_txe);
    (void)One::busy();
    (void)One::host();
    (void)One::transmitting();
    (void)One::second_address_matched();
    (void)One::general_call_matched();
    (void)One::smbus_host_matched();
    (void)One::smbus_default_matched();
    (void)One::pec_value();
    (void)One::clear_addr();
    One::clear_stopf();
    One::clear_errors(i2c_all_errors);
    One::clear_errors(i2c_af);

    One::event_interrupt(true);
    One::buffer_interrupt(true);
    One::error_interrupt(true);
    (void)One::data_address();
    (void)One::bus_hz(SysClock{});
    One::disable();
    One::bus_clock(false);

    // The second instance wherever the part has one - the same verbs.
    Two::bus_clock(true);
    Two::reset();
    Two::timing(*fm);
    Two::enable();
    (void)Two::status1();
    Two::disable();
    Two::bus_clock(false);
}

// ---- the host engine, plain and with its two DMA slots ---------------------

using Host = I2cHost<1>;
using DmaHost = I2cHost<1, i2c_default_pins<1>, DmaTxEngine<I2c<1>::dma_tx_channel>,
                        DmaRxEngine<I2c<1>::dma_rx_channel>>;
using SecondHost = I2cHost<second_i2c, i2c_default_pins<second_i2c>>;

static_assert(!Host::has_engines && DmaHost::has_engines);
static_assert(Host::pin_pads.scl == Pad{'B', 6});

uint8_t out_bytes[8];
uint8_t in_bytes[8];

void host_engine() {
    constexpr SysClock clock{};
    (void)Host::init(clock);
    (void)Host::init(clock, I2cDuty::ratio_16_9);
    (void)Host::init(clock, I2cDuty::ratio_2, 1000);
    Host::rebase(SysClock::hz);
    (void)Host::speed_ok(I2cSpeed::fast_400k);
    (void)Host::timing_of(I2cSpeed::standard_100k);
    (void)Host::scl_hz(I2cSpeed::standard_100k);
    (void)Host::reference_hz();
    (void)Host::busy();

    Host::Request r{};
    r.addr = 0x2C;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_bytes));
    r.tx_len = 4;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(in_bytes));
    r.rx_len = 4;
    r.speed = I2cSpeed::fast_400k;
    (void)Host::start(r);
    (void)Host::status();
    (void)Host::isr();
    (void)Host::error_isr();
    (void)Host::dma_isr();
    (void)Host::unstick();
    (void)Host::recover();
    Host::release();

    (void)DmaHost::init(clock);
    (void)DmaHost::start(DmaHost::Request{.addr = 0x2C,
                                          .tx = lend<Lease::reply>(
                                              static_cast<const uint8_t*>(out_bytes)),
                                          .tx_len = 8,
                                          .rx = {},
                                          .rx_len = 0,
                                          .reply = {},
                                          .speed = I2cSpeed::standard_100k});
    (void)DmaHost::isr();
    (void)DmaHost::dma_isr();
    DmaHost::release();

    (void)SecondHost::init(clock);
    SecondHost::release();
}

// ---- the client ------------------------------------------------------------

using Client = I2cClient<1>;
using SecondClient = I2cClient<second_i2c, i2c_default_pins<second_i2c>>;

void client() {
    constexpr SysClock clock{};
    (void)Client::init(clock, {.own = 0x2C});
    (void)Client::init(clock, {.own = 0x2C, .second = 0x39, .general_call = true},
                       {.no_stretch = true, .interrupts = false});
    (void)Client::addressed();
    (void)Client::answer_address();
    (void)Client::second_address_matched();
    (void)Client::general_call_matched();
    (void)Client::data_ready();
    (void)Client::take();
    (void)Client::data_wanted();
    Client::give(0x5A);
    Client::acknowledge(false);
    (void)Client::stop_seen();
    Client::clear_stop();
    (void)Client::host_nacked();
    Client::clear_nack();
    (void)Client::overrun();
    Client::clear_overrun();
    (void)Client::flags();
    (void)Client::flush();
    (void)Client::host_reads();
    const I2cClientEvent e = Client::service();
    const I2cClientEvent f = Client::error_service();
    (void)(e == I2cClientEvent::addressed || f == I2cClientEvent::nacked);
    Client::release();

    (void)SecondClient::init(clock, {.own = 0x2C});
    SecondClient::release();
}

// ---- the arbiter over the engine -------------------------------------------

class Probe {
public:
    using Event = std::variant<I2cDone, SleepVote>;
    static inline EventQueue<Event, 4, P> queue;
    static void init() {}
    static void dispatch(const Event& e) {
        match(
            e, [](const I2cDone& d) { (void)d.status; },
            [](const SleepVote& v) { (void)v.ok; });
    }
};

using I2cArb = I2cBus<Host, P, 4>;
using TimedI2cArb = I2cBus<Host, P, 4, BusPassThrough, 20>;
using Kernel = Tenuto<P, Probe, I2cArb>;

void arbiter() {
    Kernel::init_all();
    Host::Request r{};
    r.addr = 0x2C;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_bytes));
    r.tx_len = 2;
    r.reply = reply_to<Probe, I2cDone>();
    post<I2cArb>(r);
    post<I2cArb>(TransferDone{i2c_ok});
    post<I2cArb>(PrepareSleep{.depth = SleepDepth::standby,
                              .reply = reply_to<Probe, SleepVote>()});
    (void)Kernel::step();
    (void)TimedI2cArb::stale_events();
}

int main() {
    resource();
    host_engine();
    client();
    arbiter();
    return 0;
}
