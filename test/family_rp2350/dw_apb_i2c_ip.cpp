// THE IP STRATUM THROUGH THE HAZARD3 COMPILER, over a chip that is not a
// chip: brio/dw_apb_i2c/i2c.hpp instantiated on brio/host/sim_dw_apb_i2c.hpp
// - two register blocks in RAM, a reset that fills them with the block's
// reset values, an interrupt controller that counts, pads over a modelled
// wire and a ruler that counts microseconds instead of waiting. The
// RP2040's fixture makes the same proof through one compiler; this one
// makes it through two, the Cortex-M33's and the Hazard3's, which is the
// thing this target adds - a file that had smuggled in anything of a
// Cortex-M, of an NVIC or of a vendor header would fail here and nowhere
// else.
//
// Every verb of the resource, of the host engine and of the client is
// named, with both engine slots empty and both filled; the arithmetic is
// judged at the rate THIS chip runs, since the IP file's timing is pure
// arithmetic and owes no chip an answer.
#include "dw_apb_i2c/i2c.hpp"
#include "host/sim_dw_apb_i2c.hpp"

using namespace brio;

using Clock = SimDwApbI2cClock<150'000'000>;

constexpr SimDwApbI2cPins pins0{.scl = 1, .sda = 0};
constexpr SimDwApbI2cPins pins1{.scl = 3, .sda = 2};
static_assert(SimDwApbI2c::pins_valid(0, pins0) && SimDwApbI2c::pins_valid(1, pins1));
static_assert(!SimDwApbI2c::pins_valid(0, {.scl = 4, .sda = 4}));

using Block = DwApbI2cBlock<SimDwApbI2c, 1>;
using Plain = DwApbI2cHost<SimDwApbI2c, 0, pins0, SimI2cNoEngine, SimI2cNoEngine>;
using Engined = DwApbI2cHost<SimDwApbI2c, 1, pins1, SimDwApbI2cEngine<0, uint16_t>,
                             SimDwApbI2cEngine<1, uint8_t>>;
using Client = DwApbI2cClient<SimDwApbI2c, 1, pins1>;

static_assert(Block::fifo_depth == 16u);
static_assert(Block::index == 1u);
static_assert(Block::dreq_tx == SimDwApbI2cRequest::i2c1_tx && Block::dreq_rx == SimDwApbI2cRequest::i2c1_rx);
static_assert(!Plain::has_engines && Engined::has_engines);
static_assert(DwApbI2cChipClock<SimDwApbI2c, Clock>);
static_assert(std::is_trivially_copyable_v<Plain::Request>);
static_assert(sizeof(Plain::Request) == sizeof(Engined::Request));

uint8_t buf[16];

void dw_apb_i2c_resource_verbs() {
    (void)Block::regs().IC_CON;
    (void)Block::irq();
    (void)Block::data_address();
    (void)Block::reset();
    (void)Block::released();
    Block::hold();
    (void)Block::enabled();
    (void)Block::running();
    Block::enable();
    (void)Block::disable();
    Block::abort();
    (void)Block::aborting();
    (void)Block::client_disabled_while_busy();
    (void)Block::client_rx_data_lost();
    (void)Block::configure({.role = I2cRole::client, .speed = I2cSpeed::fast_400k, .ten_bit_client = true});
    (void)Block::con();
    (void)Block::timing(*i2c_timing_for(Clock::hz, I2cSpeed::fast_400k));
    (void)Block::sda_rx_hold(2);
    (void)Block::timing(I2cSpeed::fast_400k);
    (void)Block::target(0x48);
    (void)Block::target(0, true);
    (void)Block::target();
    (void)Block::own_address(0x42);
    (void)Block::own_address();
    Block::ack_general_call(true);
    (void)Block::nack_data(true);
    Block::rx_threshold(3);
    Block::tx_threshold(8);
    (void)Block::flags();
    (void)Block::tx_not_full();
    (void)Block::tx_empty();
    (void)Block::rx_not_empty();
    (void)Block::rx_full();
    (void)Block::activity();
    (void)Block::host_active();
    (void)Block::client_active();
    (void)Block::tx_count();
    (void)Block::rx_count();
    Block::push(i2c_write_entry(0x55));
    (void)Block::pop();
    (void)Block::first_data_byte();
    Block::flush_rx();
    Block::interrupts(I2cInterrupt::rx_full, true);
    Block::interrupts_only(I2cInterrupt::all);
    (void)Block::interrupts();
    (void)Block::raw_pending();
    (void)Block::pending();
    Block::clear_pending(I2cInterrupt::all);
    Block::clear_all();
    (void)Block::abort_source();
    (void)Block::flushed_entries();
    (void)Block::clear_abort();
    (void)Block::isr();
    Block::dma_requests(true, true);
    Block::dma_levels(0, 0);
    Block::command_block(false);
}

void dw_apb_i2c_host_verbs() {
    constexpr Clock clock;
    (void)Plain::init(clock);
    Plain::rebase(48'000'000);
    (void)Plain::speed_ok(I2cSpeed::fast_plus_1m);
    (void)Plain::timing_of(I2cSpeed::fast_400k);
    (void)Plain::scl_hz(I2cSpeed::standard_100k);
    (void)Plain::reference_hz();
    (void)Plain::idle();
    Plain::Request r{};
    r.addr = 0x42;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    r.tx_len = 2;
    r.rx = lend<Lease::reply>(buf);
    r.rx_len = 8;
    r.speed = I2cSpeed::fast_400k;
    (void)Plain::start(r);
    (void)Plain::isr();
    (void)Plain::dma_isr();
    (void)Plain::status();
    (void)Plain::unstick();
    (void)Plain::recover();
    Plain::release();
}

void dw_apb_i2c_engine_verbs() {
    constexpr Clock clock;
    (void)Engined::init(clock);
    Engined::Request d{};
    d.addr = 0x42;
    d.rx = lend<Lease::reply>(buf);
    d.rx_len = 16;
    (void)Engined::start(d);
    (void)Engined::isr();
    (void)Engined::dma_isr();
    (void)Engined::recover();
    Engined::release();
}

void dw_apb_i2c_client_verbs() {
    constexpr Clock clock;
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

// The IP's own arithmetic and vocabulary, judged where no chip can colour
// them - and at 150 MHz, which is this target's clk_sys and not the other
// chip's 125.
static_assert(i2c_speed_hz(I2cSpeed::fast_plus_1m) == 1'000'000u && i2c_speed_count == 3u);
static_assert(i2c_speed_code(I2cSpeed::standard_100k) == 1u && i2c_speed_code(I2cSpeed::fast_400k) == 2u);
static_assert(i2c_min_clk_hz(I2cSpeed::fast_plus_1m) == 32'000'000u);
static_assert(i2c_cycles_ceil(150'000'000, 50) == 8u && i2c_cycles_ceil(1'000'000, 1000) == 1u);
static_assert(!i2c_timing_for(1'000'000, I2cSpeed::standard_100k).has_value());
static_assert(i2c_scl_hz(150'000'000, *i2c_timing_for(150'000'000, I2cSpeed::standard_100k)) == 100'000u);
static_assert(i2c_scl_hz(150'000'000, *i2c_timing_for(150'000'000, I2cSpeed::fast_plus_1m)) == 1'000'000u);
static_assert(i2c_write_entry(0x12, {.restart = true}) == 0x0412u && i2c_read_entry({.stop = true}) == 0x0300u);
static_assert(i2c_con_of({.role = I2cRole::client}) == ((I2cControl::speed_standard << I2cControl::speed_lsb) |
                                                        I2cControl::restart_enable |
                                                        I2cControl::stop_det_if_addressed |
                                                        I2cControl::tx_empty_late |
                                                        I2cControl::hold_when_rx_full));
static_assert(i2c_status_of_abort(I2cAbort::client_flush) == i2c_bus_error);
static_assert(I2cInterrupt::all == 0x1FFFu);
static_assert(i2c_dma_fault == bus_engine_status + 4);
