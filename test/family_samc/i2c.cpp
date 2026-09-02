// SERCOM (I2C mode) family smoke TU: the I2cm<n>/I2cs<n> resources, the
// I2cHost and I2cClient tasks over them, and the chapter's rise-time
// baud arithmetic.
//
// SERCOM0 is what every variant of the pack has, and PA08/PA09 reach
// its PAD[0]/PAD[1] through PMUX function C on the E, the G and the J
// alike - AND they are on table 6-7's I2C-capable list for every
// package, which the bench pair PA22/PA23 (SERCOM3, J-board fact) also
// is. The board's own pads are an app-level fact and not compiled here.
#include "samc/clock.hpp"
#include "samc/i2c.hpp"
#include "samc/platform_sam.hpp"
#include "kernel/time.hpp"
#include "util/i2c_bus.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 48'000'000>;

constexpr I2cPads pads{
    .sda_pin = {'A', 8, PinFunction::c},
    .scl_pin = {'A', 9, PinFunction::c},
};

// ---- the arithmetic, beyond the header's own pins --------------------------
// A slower core clock still reaches 100 kHz; the rise time eats real
// cycles (at 8 MHz, 300 ns is 2 of them).
static_assert(i2c_baud_for(8'000'000UL, 100'000UL, 300u, false).has_value());
static_assert(i2c_scl_hz(8'000'000UL, *i2c_baud_for(8'000'000UL, 100'000UL, 300u, false),
                         300u) <= 100'000UL);
// The produced rate is never above the request, whatever the rounding.
static_assert(i2c_scl_hz(48'000'000UL, *i2c_baud_for(48'000'000UL, 400'000UL, 300u, false),
                         300u) <= 400'000UL);
// Fm+ splits 1:2 - the LOW half is never the shorter one.
static_assert(i2c_baud_for(48'000'000UL, 1'000'000UL, 120u, true)->baudlow >=
              i2c_baud_for(48'000'000UL, 1'000'000UL, 120u, true)->baud);

// ---- the refusals, as VALUES (the compile-time twins are the negatives) ----
static_assert(i2cm_config_valid([] {
    I2cmConfig c{};
    c.pads = pads;
    c.baud = *i2c_baud_for(48'000'000UL, 100'000UL, 300u, false);
    return c;
}()));
// Erratum 1.17.13: quick command and SCLSM = 1 cannot travel together.
static_assert(!i2cm_config_valid([] {
    I2cmConfig c{};
    c.pads = pads;
    c.baud = I2cBaud{100, 0};
    c.quick_command = true;
    c.scl_stretch_after_ack = true;
    return c;
}()));
// The chapter's own note: BAUD and/or BAUDLOW must be nonzero.
static_assert(!i2cm_config_valid([] {
    I2cmConfig c{};
    c.pads = pads;
    return c;
}()));
// An 8-bit client address is not a 7-bit address.
static_assert(!i2cs_config_valid([] {
    I2csConfig c{};
    c.pads = pads;
    c.address = 0x80;
    return c;
}()));
// A range with inverted bounds matches nothing 33.8.2 describes.
static_assert(!i2cs_config_valid([] {
    I2csConfig c{};
    c.pads = pads;
    c.address_mode = I2cAddressMode::range;
    c.address = 0x10;
    c.second = 0x20;
    return c;
}()));

// ---- erratum 1.17.8 as construction ----------------------------------------
// The W1C masks cannot name CLKHOLD: bit 7 is absent from both.
static_assert((I2cmStatus::w1c_all & SERCOM_I2CM_STATUS_CLKHOLD_Msk) == 0u);
static_assert((I2csStatus::w1c_all & SERCOM_I2CS_STATUS_CLKHOLD_Msk) == 0u);

// ---- the two register views really differ ----------------------------------
// The SPI's SPIM/SPIS equivalence does NOT hold here - which is why
// i2c.hpp carries TWO resources where spi.hpp carries one. Held so a
// reader who pattern-matches from the SPI file learns otherwise at
// compile time.
static_assert(SERCOM_I2CM_INTFLAG_MB_Msk == 0x01u);
static_assert(SERCOM_I2CS_INTFLAG_PREC_Msk == 0x01u);   // same bit, different meaning

// ---- the tasks instantiate on every variant --------------------------------
using Host = I2cHost<0, pads>;
using Client = I2cClient<0, pads>;

bool bring_up_host() {
    constexpr SysClock clock{};
    return Host::init(clock, 300u);
}

bool one_tenure(uint8_t addr, const uint8_t* cmd, uint8_t n, uint8_t* in, uint8_t m) {
    Host::Request r{
        .addr = addr,
        .tx = lend<Lease::reply>(cmd),
        .tx_len = n,
        .rx = lend<Lease::reply>(in),
        .rx_len = m,
        .reply = {},
        .speed = I2cSpeed::fast_400k,
    };
    return Host::start(r);
}

bool host_isr_body() { return Host::isr(); }
uint8_t host_status() { return Host::status(); }
uint8_t wire_fix() { return Host::unstick(); }

bool bring_up_client() {
    constexpr SysClock clock{};
    return Client::init(clock, {.address_mode = I2cAddressMode::mask,
                                .address = 0x2C,
                                .general_call = true});
}

void client_turnaround() {
    if (Client::addressed()) {
        Client::answer_address(true);
    }
    if (Client::data_ready()) {
        if (Client::host_reads()) {
            Client::give(0x5A);
            Client::end_transaction();
            (void)Client::first_drdy();   // erratum 1.17.22's gate
        } else {
            (void)Client::take(true);
        }
    }
    if (Client::stop_seen()) {
        Client::clear_stop();
    }
}

uint8_t client_isr_body() { return Client::isr(); }

// The bus-state vocabulary is the host resource's.
I2cBusState state_now() { return I2cm<0>::bus_state(); }

// ---- the per-bus timeout instantiates over this engine ---------------------
// A timed I2cBus static_asserts Bus::recover(); this engine's re-runs
// the init() tail from the cached configuration, because a START
// parked into a held wire never fires on this silicon and only a
// re-init gets out (util/bus_master.hpp, docs/samc/i2c.md).
using TimedI2cBus = I2cBus<Host, SamPlatform, 4, BusPassThrough,
                           ticks_from_ms<SamPlatform>(250)>;

void timed_bus_surface() {
    TimedI2cBus::init();
    (void)TimedI2cBus::rejected_count();
    (void)TimedI2cBus::stale_events();
    (void)Host::recover();
}
