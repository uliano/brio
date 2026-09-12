// I2C family smoke TU: the vocabulary and its arithmetic, the pin
// table, the resource's verbs, a host with and without engines, a
// client.
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/i2c.hpp"

using namespace brio;

static_assert(i2c_speed_hz(I2cSpeed::fast_plus_1m) == 1'000'000u && i2c_speed_count == 3u);
static_assert(i2c_speed_code(I2cSpeed::standard_100k) == 1u && i2c_speed_code(I2cSpeed::fast_plus_1m) == 2u);
static_assert(i2c_min_clk_hz(I2cSpeed::fast_plus_1m) == 32'000'000u);
static_assert(i2c_timing_for(125'000'000, I2cSpeed::standard_100k).has_value());
static_assert(!i2c_timing_for(1'000'000, I2cSpeed::standard_100k).has_value());
static_assert(i2c_scl_hz(125'000'000, *i2c_timing_for(125'000'000, I2cSpeed::fast_plus_1m)) == 1'000'000u);
static_assert(i2c_sda_pin(0, 12) && i2c_scl_pin(0, 13) && i2c_sda_pin(1, 14) && i2c_scl_pin(1, 15));
static_assert(!i2c_sda_pin(1, 12) && !i2c_scl_pin(0, 15));
static_assert(i2c_pins_valid(0, {.scl = 13, .sda = 12}) && !i2c_pins_valid(0, {.scl = 15, .sda = 14}));
static_assert((i2c_con_of({.role = I2cRole::client}) & (I2C_IC_CON_MASTER_MODE_BITS | I2C_IC_CON_IC_SLAVE_DISABLE_BITS)) == 0u);
static_assert(i2c_write_entry(0x12, {.restart = true}) == 0x0412u && i2c_read_entry({.stop = true}) == 0x0300u);
static_assert(i2c_status_of_abort(I2cAbort::client_flush) == i2c_bus_error);
static_assert(DwApbI2c<1>::dreq_rx == Dreq::i2c1_rx && DwApbI2c<0>::fifo_depth == 16u);

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr I2cPins host_pins{.scl = 13, .sda = 12};
constexpr I2cPins client_pins{.scl = 15, .sda = 14};
using Host = I2cHost<0, host_pins>;
using DmaHost = I2cHost<0, host_pins, DmaTxEngine<4, uint16_t>, DmaRxEngine<5>>;
using Client = I2cClient<1, client_pins>;
static_assert(!Host::has_engines && DmaHost::has_engines);
static_assert(std::is_trivially_copyable_v<Host::Request>);
static_assert(sizeof(Host::Request) == sizeof(DmaHost::Request));

uint8_t buf[16];

void i2c_verbs() {
    constexpr SysClock clock;
    using S = DwApbI2c<0>;
    (void)S::reset();
    (void)S::released();
    S::hold();
    (void)S::enabled();
    (void)S::running();
    S::enable();
    (void)S::disable();
    S::abort();
    (void)S::aborting();
    (void)S::client_disabled_while_busy();
    (void)S::client_rx_data_lost();
    (void)S::configure({.role = I2cRole::client, .speed = I2cSpeed::fast_400k, .ten_bit_client = true});
    (void)S::con();
    (void)S::timing(*i2c_timing_for(SysClock::hz, I2cSpeed::fast_400k));
    (void)S::sda_rx_hold(2);
    (void)S::timing(I2cSpeed::fast_400k);
    (void)S::target(0x48);
    (void)S::target(0, true);
    (void)S::target();
    (void)S::own_address(0x42);
    (void)S::own_address();
    S::ack_general_call(true);
    (void)S::nack_data(true);
    S::rx_threshold(3);
    S::tx_threshold(8);
    (void)S::flags();
    (void)S::tx_not_full();
    (void)S::tx_empty();
    (void)S::rx_not_empty();
    (void)S::rx_full();
    (void)S::activity();
    (void)S::host_active();
    (void)S::client_active();
    (void)S::tx_count();
    (void)S::rx_count();
    S::push(i2c_write_entry(0x55));
    (void)S::pop();
    (void)S::first_data_byte();
    S::flush_rx();
    S::interrupts(I2cInterrupt::rx_full, true);
    S::interrupts_only(I2cInterrupt::all);
    (void)S::interrupts();
    (void)S::raw_pending();
    (void)S::pending();
    S::clear_pending(I2cInterrupt::all);
    S::clear_all();
    (void)S::abort_source();
    (void)S::flushed_entries();
    (void)S::clear_abort();
    (void)S::isr();
    S::dma_requests(true, true);
    S::dma_levels(0, 0);
    S::command_block(false);

    (void)Host::init(clock);
    Host::rebase(48'000'000);
    (void)Host::speed_ok(I2cSpeed::fast_plus_1m);
    (void)Host::timing_of(I2cSpeed::fast_400k);
    (void)Host::scl_hz(I2cSpeed::standard_100k);
    (void)Host::reference_hz();
    (void)Host::idle();
    Host::Request r{};
    r.addr = 0x42;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    r.tx_len = 2;
    r.rx = lend<Lease::reply>(buf);
    r.rx_len = 8;
    r.speed = I2cSpeed::fast_400k;
    (void)Host::start(r);
    (void)Host::isr();
    (void)Host::dma_isr();
    (void)Host::status();
    (void)Host::unstick();
    (void)Host::recover();
    Host::release();
    (void)DmaHost::init(clock);
    DmaHost::Request d{};
    d.addr = 0x42;
    d.rx = lend<Lease::reply>(buf);
    d.rx_len = 16;
    (void)DmaHost::start(d);
    (void)DmaHost::isr();
    (void)DmaHost::dma_isr();
    (void)DmaHost::recover();
    DmaHost::release();

    (void)Client::init(clock, {.address = 0x42, .general_call = true, .speed = I2cSpeed::fast_plus_1m});
    Client::interrupts(Client::events, true);
    (void)Client::data_ready();
    (void)Client::take();
    (void)Client::data_wanted();
    Client::give(0x5A);
    (void)Client::writable();
    Client::clear_read_request();
    (void)Client::stop_seen();
    Client::clear_stop();
    (void)Client::host_nacked();
    Client::clear_nack();
    (void)Client::overrun();
    Client::clear_overrun();
    (void)Client::general_call_seen();
    (void)Client::flags();
    (void)Client::raw_pending();
    (void)Client::acknowledge(false);
    Client::flush_tx();
    (void)Client::service();
    (void)Client::host_reads();
    Client::release();
}
