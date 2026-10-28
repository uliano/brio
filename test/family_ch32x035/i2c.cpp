// I2C family smoke TU (RM ch. 15), every part: the map against table
// 15-1, I2C1's six columns and which of them each package can use, the
// resource, the host on its default column in both engine arrangements
// and under util/i2c_bus.hpp's arbiter, the client polled and served from
// its vectors - instantiation only, no main(), no hardware. The clock
// arithmetic is pinned in the header; what this fixture adds is the
// columns per package and the util contract, unchanged on this core.
#include <stddef.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/i2c.hpp"
#include "ch32x035/platform.hpp"
#include "kernel/tenuto.hpp"
#include "util/i2c_bus.hpp"

using namespace brio;

using P = Ch32x035Platform<>;
using SysClock = Clock<ClockSource::internal, 48'000'000>;
using Slow = Clock<ClockSource::internal, 8'000'000>;

// ---- the map (table 15-1) ---------------------------------------------------
static_assert(i2c_base_for(1) == 0x40005400 && i2c_base_for(2) == 0);
static_assert(offsetof(I2cRegs, CTLR2) == 0x04 && offsetof(I2cRegs, DATAR) == 0x10 &&
              offsetof(I2cRegs, STAR2) == 0x18 && offsetof(I2cRegs, CKCFGR) == 0x1c &&
              sizeof(I2cRegs) == 0x20, "the block ends at CKCFGR: no rise-time register");
static_assert(I2c<1>::dma_tx_channel == 6 && I2c<1>::dma_rx_channel == 7);
static_assert(static_cast<uint8_t>(I2c<1>::event_irq()) == 30 &&
              static_cast<uint8_t>(I2c<1>::error_irq()) == 31);
static_assert(i2c_dma_fault > i2c_bus_error && i2c_dma_fault < bus_timeout,
              "the engine's own code sits above i2c_bus.hpp's four and below the arbiter's timeout");

// ---- the columns per package ----------------------------------------------------
// PA10/PA11 where the package bonds them, PC16/PC17 only where they are
// alone on their pins (the QFN20 CH32X035F8), the debug port's pair where
// nothing else is left.
constexpr bool bonded_alone(char port, uint8_t pin) { return pad_can_drive(Pad{port, pin}); }
static_assert(i2c_pins_valid(i2c1_pins_for(0)) == (bonded_alone('A', 10) && bonded_alone('A', 11)));
static_assert(i2c_pins_valid(i2c1_pins_for(2)) == (bonded_alone('C', 16) && bonded_alone('C', 17)));
static_assert(i2c_pins_valid(i2c1_pins_for(2)) == i2c_pins_valid(i2c1_pins_for(4)));
static_assert(i2c_pins_valid(i2c1_pins_for(3)) && i2c_pins_valid(i2c1_pins_for(5)),
              "every package of the series bonds PC18 and PC19 alone");
static_assert(i2c_pins_valid(i2c1_default_pins));
static_assert(device::package != Package::qfn20 || device::flash_bytes == 0 ||
              i2c1_default_pins.remap == (i2c_pins_valid(i2c1_pins_for(0)) ? 0u : 2u) ||
              i2c_pins_on_debug_port(i2c1_default_pins));
static_assert(!i2c_pins_on_debug_port(i2c1_default_pins) ||
              (!i2c_pins_valid(i2c1_pins_for(0)) && !i2c_pins_valid(i2c1_pins_for(1)) &&
               !i2c_pins_valid(i2c1_pins_for(2))),
              "the default is the debug port's pair only where no other column exists");
static_assert(afio_remap_has_code(Remap::i2c1, 3) && !afio_remap_has_code(Remap::i2c1, 8));

using Host = I2cHost<1>;
using HostDma = I2cHost<1, i2c1_default_pins, DmaTxEngine<6>, DmaRxEngine<7>>;
using Client = I2cClient<1>;
// The debug port's column, on every part: compiled, refused at run time
// while the probe owns the pads.
using DebugHost = I2cHost<1, i2c1_pins_for(5)>;
// Both placements (util/placement.hpp): an engined host whose event pump
// runs from SRAM.
using RamHostDma = I2cHost<1, i2c1_default_pins, DmaTxEngine<6>, DmaRxEngine<7>, InRam>;

static_assert(!Host::has_engines && HostDma::has_engines);
static_assert(std::is_trivially_copyable_v<Host::Request>);

using PlainBus = I2cBus<Host, P, 4>;
using TimedBus = I2cBus<HostDma, P, 2, BusPassThrough, 50>;

void resource_verbs() {
    using S = I2c<1>;
    (void)S::present();
    S::bus_clock(true);
    S::reset();
    (void)S::remap(i2c1_default_pins.remap);
    S::timing(*i2c_timing_for(48'000'000, I2cSpeed::fast_400k));
    (void)S::addresses({.own = 0x48, .second = 0x49, .general_call = true});
    S::enable(); (void)S::enabled();
    S::software_reset();
    S::start(); (void)S::starting(); S::stop(); (void)S::stopping();
    S::ack(true); S::pos(true); S::general_call(false); S::no_stretch(false);
    S::pec(false); S::pec_next(false); S::arp(false);
    S::dma(true, true);
    S::data(0x55); (void)S::data();
    (void)S::status1(); (void)S::status2(); (void)S::flag(i2c_btf);
    (void)S::busy(); (void)S::host(); (void)S::transmitting(); (void)S::pec_value();
    (void)S::second_address_matched(); (void)S::general_call_matched();
    (void)S::clear_addr(); S::clear_stopf(); S::clear_errors(i2c_errors);
    S::event_interrupt(true); S::buffer_interrupt(true); S::error_interrupt(true);
    S::control2(48, i2c_iterren | i2c_itevten);
    S::disable();
    (void)Afio::usb_pads_taken();
    Afio::usb_pad_pull_ups(UsbPadPull::r1k5, UsbPadPull::weak);
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
    (void)H::busy();
    (void)H::unwedges();
    (void)H::unwedge_busy();
    (void)H::unstick();
    (void)H::recover();
    H::release();
}

void all_hosts() {
    static uint8_t buf[16];
    host_verbs<Host>(buf);
    host_verbs<HostDma>(buf);
    host_verbs<DebugHost>(buf);
    host_verbs<RamHostDma>(buf);
}

void client_verbs() {
    constexpr Slow clock;
    (void)Client::init(clock, {.own = 0x48, .second = 0x49});
    (void)Client::init(clock, {.own = 0x2C}, {.no_stretch = false, .interrupts = false});
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
    (void)Client::flush();
    (void)Client::service();
    (void)Client::error_service();
    (void)Client::host_reads();
    Client::release();
}

using Loop = Tenuto<P, PlainBus>;
using TimedLoop = Tenuto<P, TimedBus>;

void arbiters() {
    Loop::init_all();
    Loop::step();
    TimedLoop::init_all();
    TimedLoop::step();
    post<PlainBus>(TransferDone{i2c_nack_addr});
    post<TimedBus>(TransferDone{i2c_dma_fault});
}

// The app's glue: the two vectors and the two channels, bound both ways
// the hardware prologue can be built - the event vector placed in SRAM by
// the program (`.fast_text`) beside the InRam host's trampoline
// (`.fast_text.brio`) in this unit.
extern "C" [[gnu::section(".fast_text")]] void i2c1_ev_handler();
BRIO_CH32_VECTOR(i2c1_ev_handler) {
    if (HostDma::isr()) {
        post<TimedBus>(TransferDone{HostDma::status()});
    }
    (void)RamHostDma::isr();
}
BRIO_CH32_VECTOR(i2c1_er_handler) {
    if (HostDma::error_isr()) {
        post<TimedBus>(TransferDone{HostDma::status()});
    }
}
BRIO_CH32_VECTOR(dma1_channel7_handler) {
    if (HostDma::dma_isr()) {
        post<TimedBus>(TransferDone{HostDma::status()});
    }
}
