// I2C family smoke TU: ch32v00x/i2c.hpp's resource, host and client
// instantiated, the host in both engine arrangements and under the
// I2cBus arbiter of util/i2c_bus.hpp - instantiation only, no main(),
// no hardware. The clock arithmetic is pinned in the header; what this
// fixture adds is the util contract, unchanged on this architecture.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/i2c.hpp"
#include "ch32v00x/platform.hpp"
#include "kernel/kernel.hpp"
#include "util/i2c_bus.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;
using Slow = Clock<ClockSource::internal, 24'000'000>;

static_assert(i2c_base_for(1) == 0x40005400 && i2c_base_for(2) == 0);
static_assert(I2c<1>::dma_tx_channel == 6 && I2c<1>::dma_rx_channel == 7);
static_assert(offsetof(I2cRegs, CKCFGR) == 0x1c);
static_assert(i2c_dma_fault > i2c_bus_error && i2c_dma_fault < bus_timeout,
              "the engine's own code sits above i2c_bus.hpp's four and below the arbiter's timeout");

using Host = I2cHost<1>;
using HostDma = I2cHost<1, i2c1_default_pins, DmaTxEngine<6>, DmaRxEngine<7>>;
using Client = I2cClient<1>;

static_assert(!Host::has_engines && HostDma::has_engines);
static_assert(std::is_trivially_copyable_v<Host::Request>);

using PlainBus = I2cBus<Host, P, 4>;
using TimedBus = I2cBus<HostDma, P, 2, BusPassThrough, 50>;

void resource_verbs() {
    using S = I2c<1>;
    S::bus_clock(true);
    S::reset();
    S::timing(*i2c_timing_for(48'000'000, I2cSpeed::fast_400k));
    (void)S::addresses({.own = 0x48, .second = 0x49, .general_call = true});
    S::enable(); (void)S::enabled();
    S::software_reset();
    S::start(); (void)S::starting(); S::stop(); (void)S::stopping();
    S::ack(true); S::pos(true); S::general_call(false); S::no_stretch(false); S::pec(false);
    S::dma(true, true);
    S::data(0x55); (void)S::data();
    (void)S::status1(); (void)S::status2(); (void)S::flag(i2c_btf);
    (void)S::busy(); (void)S::host(); (void)S::transmitting();
    (void)S::second_address_matched(); (void)S::general_call_matched();
    (void)S::clear_addr(); S::clear_stopf(); S::clear_errors(i2c_errors);
    S::event_interrupt(true); S::buffer_interrupt(true); S::error_interrupt(true);
    S::disable();
}

template <typename H>
void host_verbs(uint8_t* buf) {
    constexpr SysClock clock;
    (void)H::init(clock, I2cDuty::ratio_16_9);
    H::rebase(24'000'000);
    (void)H::speed_ok(I2cSpeed::fast_400k);
    (void)H::timing_of(I2cSpeed::standard_100k);
    (void)H::scl_hz(I2cSpeed::fast_400k);
    (void)H::reference_hz();
    typename H::Request r{};
    r.addr = 0x48;
    r.tx = Borrowed<const uint8_t, Lease::reply>{buf};
    r.tx_len = 2;
    r.rx = Borrowed<uint8_t, Lease::reply>{buf + 8};
    r.rx_len = 4;
    r.speed = I2cSpeed::fast_400k;
    (void)H::start(r);
    (void)H::isr();
    (void)H::error_isr();
    (void)H::dma_isr();
    (void)H::status();
    (void)H::unstick();
    (void)H::recover();
    H::release();
}

void all_hosts() {
    static uint8_t buf[16];
    host_verbs<Host>(buf);
    host_verbs<HostDma>(buf);
}

void client_verbs() {
    constexpr Slow clock;
    (void)Client::init(clock, {.own = 0x48, .second = 0x49}, false);
    (void)Client::addressed();
    (void)Client::answer_address();
    (void)Client::second_address_matched();
    (void)Client::general_call_matched();
    (void)Client::data_ready(); (void)Client::take();
    (void)Client::data_wanted(); Client::give(0xAA);
    Client::acknowledge(true);
    (void)Client::stop_seen(); Client::clear_stop();
    (void)Client::host_nacked(); Client::clear_nack();
    (void)Client::overrun(); Client::clear_overrun();
    (void)Client::flags();
    (void)Client::service();
    (void)Client::error_service();
    (void)Client::host_reads();
    Client::release();
}

using Loop = Kernel<P, PlainBus>;
using TimedLoop = Kernel<P, TimedBus>;

void arbiters() {
    Loop::init_all();
    Loop::step();
    TimedLoop::init_all();
    TimedLoop::step();
    post<PlainBus>(TransferDone{i2c_nack_addr});
    post<TimedBus>(TransferDone{i2c_dma_fault});
}
