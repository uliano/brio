// I2C family smoke TU: this chip's half of the DW_apb_i2c contract - the
// pin table of 9.4 over the whole of the larger bank, the chip traits
// against the IP stratum's concept, and every verb of the resource, of
// the host engine (with and without engines) and of the client. Compiled
// for both packages and both architectures, so the same source proves
// that the traits satisfy `DwApbI2cChip` whichever interrupt controller
// `Irq` names, and that the public aliases instantiate over each.
//
// The types below name GP12..GP15 alone, which both packages bond: what
// the QFN-60 refuses is a neg of its own (qfn60_i2c_past_the_package).
#include "rp2350/clock.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/i2c.hpp"

using namespace brio;

// ---- the table of 9.4, one column over forty-eight pads --------------------

// SDA on the even pads, SCL on the odd, the instance alternating in
// pairs - to the top of the bank, four pads above where the RP2040's
// table stopped.
static_assert(i2c_sda_pin(0, 0) && i2c_scl_pin(0, 1) && i2c_sda_pin(1, 2) && i2c_scl_pin(1, 3));
static_assert(i2c_sda_pin(0, 12) && i2c_scl_pin(0, 13) && i2c_sda_pin(1, 14) && i2c_scl_pin(1, 15));
static_assert(i2c_sda_pin(0, 28) && i2c_scl_pin(0, 29) && i2c_sda_pin(1, 30) && i2c_scl_pin(1, 31));
static_assert(i2c_sda_pin(0, 40) && i2c_scl_pin(0, 41) && i2c_sda_pin(1, 46) && i2c_scl_pin(1, 47));
// And nothing outside it: the other instance's pad, a line on the wrong
// parity, a pad past the die's forty-eight.
static_assert(!i2c_sda_pin(1, 12) && !i2c_scl_pin(0, 15) && !i2c_sda_pin(0, 13) && !i2c_scl_pin(1, 14));
static_assert(!i2c_sda_pin(0, 48) && !i2c_scl_pin(1, 49) && !i2c_sda_pin(0, 0xFF));
static_assert(i2c_pins_valid(0, {.scl = 13, .sda = 12}) && i2c_pins_valid(1, {.scl = 15, .sda = 14}));
static_assert(!i2c_pins_valid(0, {.scl = 15, .sda = 14}) && !i2c_pins_valid(2, {.scl = 13, .sda = 12}));

// The pad setup 12.2.1.3 asks for, and the function that routes an I2C
// to a pad: ONE code here, where the UART of this chip has two.
static_assert(i2c_pad_config.pull == PinPull::up && i2c_pad_config.schmitt &&
              !i2c_pad_config.slew_fast && i2c_pad_config.input_enable);
static_assert(Rp2350DwApbI2c::pad_function == PinFunction::i2c);
static_assert(Rp2350DwApbI2c::open_drain_pull == PinPull::up);

// ---- the traits, and the facts they carry ---------------------------------

static_assert(DwApbI2cChip<Rp2350DwApbI2c>);
static_assert(Rp2350DwApbI2c::instances == 2u && Rp2350DwApbI2c::fifo_depth == 16u);
static_assert(Rp2350DwApbI2c::irq<0>() == I2C0_IRQ_IRQn && Rp2350DwApbI2c::irq<1>() == I2C1_IRQ_IRQn);
static_assert(Rp2350DwApbI2c::tx_request<0>() == Dreq::i2c0_tx &&
              Rp2350DwApbI2c::rx_request<1>() == Dreq::i2c1_rx);
// This chip's DREQ rows, not the RP2040's 32..35.
static_assert(static_cast<uint8_t>(Dreq::i2c0_tx) == 44u && static_cast<uint8_t>(Dreq::i2c1_rx) == 47u);
// And this chip's reset bits, not the RP2040's 3 and 4.
static_assert(Rp2350DwApbI2c::reset_bit<0>() == (1UL << 4) && Rp2350DwApbI2c::reset_bit<1>() == (1UL << 5));

// ---- the IP's own arithmetic, asked at THIS chip's rate --------------------

static_assert(i2c_speed_hz(I2cSpeed::fast_plus_1m) == 1'000'000u && i2c_speed_count == 3u);
static_assert(i2c_speed_code(I2cSpeed::standard_100k) == 1u && i2c_speed_code(I2cSpeed::fast_plus_1m) == 2u);
static_assert(i2c_min_clk_hz(I2cSpeed::fast_plus_1m) == 32'000'000u);
static_assert(i2c_timing_for(150'000'000, I2cSpeed::standard_100k).has_value());
static_assert(!i2c_timing_for(1'000'000, I2cSpeed::standard_100k).has_value());
static_assert(i2c_scl_hz(150'000'000, *i2c_timing_for(150'000'000, I2cSpeed::fast_plus_1m)) == 1'000'000u);
// Table 1053's own rows, which are the same three numbers the other
// chip's table gives: the minimum clock per speed, and the counts at it.
static_assert(i2c_timing_for(12'000'000, I2cSpeed::fast_400k)->lcnt == 15u &&
              i2c_timing_for(12'000'000, I2cSpeed::fast_400k)->hcnt == 6u);
static_assert(i2c_timing_for(2'700'000, I2cSpeed::standard_100k)->lcnt == 12u);
static_assert(!i2c_timing_for(11'999'999, I2cSpeed::fast_400k).has_value());
static_assert(!i2c_timing_for(31'999'999, I2cSpeed::fast_plus_1m).has_value());
static_assert(i2c_write_entry(0x12, {.restart = true}) == 0x0412u && i2c_read_entry({.stop = true}) == 0x0300u);
static_assert(i2c_status_of_abort(I2cAbort::client_flush) == i2c_bus_error);
static_assert(DwApbI2c<1>::dreq_rx == Dreq::i2c1_rx && DwApbI2c<0>::fifo_depth == 16u);

// ---- the public names ------------------------------------------------------

using SysClock = Clock<ClockSource::pll, 150'000'000>;
constexpr I2cPins host_pins{.scl = 13, .sda = 12};
constexpr I2cPins client_pins{.scl = 15, .sda = 14};
using Host = I2cHost<0, host_pins>;
using DmaHost = I2cHost<0, host_pins, DmaTxEngine<4, uint16_t>, DmaRxEngine<5>>;
using LineHost = I2cHost<0, host_pins, DmaTxEngine<6, uint16_t, 2>, DmaRxEngine<7, uint8_t, 2>>;
using Client = I2cClient<1, client_pins>;
static_assert(!Host::has_engines && DmaHost::has_engines && LineHost::has_engines);
static_assert(std::is_trivially_copyable_v<Host::Request>);
static_assert(sizeof(Host::Request) == sizeof(DmaHost::Request));
static_assert(DwApbI2cChipClock<Rp2350DwApbI2c, SysClock>);

uint8_t buf[16];

void i2c_verbs() {
    constexpr SysClock clock;
    using S = DwApbI2c<0>;
    (void)S::regs().IC_CON;
    (void)S::irq();
    (void)S::data_address();
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

    // The same host with its engines on the THIRD of this chip's four
    // interrupt lines: the line is an engine parameter here, and a
    // transport never names it.
    (void)LineHost::init(clock);
    (void)LineHost::dma_isr();
    LineHost::release();

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

// The five verbs the unstick drives an open-drain line with, on a pad
// BOTH packages bond. Which half of the bank a pad falls in is an
// `if constexpr` on `Pin<n>::high_half` inside I2cPad, so a pad above
// GP31 takes GPIO_HI_OE and not GPIO_OE; that branch cannot be named in
// a source compiled for both packages, because the QFN-60 refuses the
// pad itself one level down - the qfn60 neg beside this file is where
// that refusal is proven.
void i2c_pad_verbs() {
    using Pad = Rp2350DwApbI2c::Pad<12>;
    (void)Pad::function(Rp2350DwApbI2c::pad_function, Rp2350DwApbI2c::pad_config);
    (void)Pad::input(Rp2350DwApbI2c::open_drain_pull);
    Pad::clear();
    (void)Pad::read();
    Pad::drive_low();
    Pad::release_drive();
    (void)Pad::release();
}
