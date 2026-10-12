// SPI family smoke TU (RM ch. 16), every part: the map against table
// 16-2, SPI1's four columns against what each package bonds, the
// resource, the host on its default column with no engine, with byte
// engines and with half-word engines, under util/spi_bus.hpp's arbiter
// untimed and timed, the client with its runs - instantiation only, no
// main(), no hardware. The rate arithmetic is pinned in the header; what
// this fixture adds is the columns per package (a host whose MOSI shares
// its pin with another pad - PA7 on the CH32X033F8P6 - is not formed) and
// the util contract, unchanged on this core.
#include <stddef.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/spi.hpp"
#include "kernel/tenuto.hpp"
#include "util/spi_bus.hpp"

using namespace brio;

using P = Ch32x035Platform<>;
using SysClock = Clock<ClockSource::internal, 48'000'000>;

// ---- the map (table 16-2) ---------------------------------------------------
static_assert(spi_base_for(1) == 0x40013000 && spi_base_for(2) == 0);
static_assert(offsetof(SpiRegs, CTLR2) == 0x04 && offsetof(SpiRegs, STATR) == 0x08 &&
              offsetof(SpiRegs, DATAR) == 0x0c && offsetof(SpiRegs, CRCR) == 0x10 &&
              offsetof(SpiRegs, RCRCR) == 0x14 && offsetof(SpiRegs, TCRCR) == 0x18 &&
              offsetof(SpiRegs, HSCR) == 0x24 && sizeof(SpiRegs) == 0x28);
static_assert(Spi<1>::dma_rx_channel == 2 && Spi<1>::dma_tx_channel == 3);
static_assert(static_cast<uint8_t>(Spi<1>::irq()) == 33, "table 7-1: SPI1 is entry 33");
static_assert(spi_dma_fault == bus_engine_status);

// ---- the columns per package ------------------------------------------------
constexpr bool column_bonded(uint8_t code) {
    const SpiPadSet c = afio_spi1_pads(code);
    return pad_bonded(c.nss) && pad_bonded(c.sck) && pad_bonded(c.miso) && pad_bonded(c.mosi);
}
static_assert(spi_pins_valid(spi1_pins_for(0)) == column_bonded(0));
static_assert(spi_pins_valid(spi1_pins_for(1)) == column_bonded(1));
static_assert(spi_pins_valid(spi1_pins_for(2)) == column_bonded(2));
static_assert(spi_pins_valid(spi1_pins_for(3)) == column_bonded(3));
static_assert(spi_pins_valid(spi1_default_pins), "every package bonds PA4..PA7");
static_assert(afio_remap_has_code(Remap::spi1, 0) && !afio_remap_has_code(Remap::spi1, 4));
static_assert(device::package != Package::qfn20 || !column_bonded(3),
              "the QFN20 bonds the first column alone");

/// The host is formed where the package lets it drive SCK and MOSI.
constexpr bool host_column_ok(const SpiPins& p) {
    return spi_pins_valid(p) && pad_can_drive(p.sck) && pad_can_drive(p.mosi);
}

void resource_verbs() {
    using S = Spi<1>;
    S::bus_clock(true);
    (void)S::bus_clock();
    S::reset();
    (void)S::configure({.role = SpiRole::host, .mode = SpiMode::mode3, .clock = SpiClock::div2,
                        .bits = SpiDataSize::bits16, .crc = true, .open_drain = true,
                        .high_speed_read = true});
    S::enable(); (void)S::enabled();
    S::data(0x1234); (void)S::data();
    (void)S::rxne(); (void)S::txe(); (void)S::busy(); (void)S::status();
    S::flush_rx();
    (void)S::overrun(); S::clear_overrun();
    (void)S::mode_fault(); S::clear_mode_fault();
    (void)S::crc_error(); S::clear_crc_error();
    S::crc_next(); (void)S::rx_crc(); (void)S::tx_crc();
    S::software_select(true);
    S::open_drain(false); (void)S::open_drain();
    (void)S::high_speed_read(true); (void)S::high_speed_read(); (void)S::hscr_register();
    S::high_speed_read_raw(false);
    S::dma_requests(true, true);
    S::rxne_interrupt(true); S::txe_interrupt(false); S::error_interrupt(true);
    (void)S::isr();
    (void)S::disable();
}

template <typename H>
void host_verbs(uint8_t* buf) {
    constexpr SysClock clock;
    (void)H::init(clock, 12'000'000);
    (void)H::init(clock, {.max_sck_hz = 0, .open_drain = true, .high_speed_read = true});
    H::rebase(24'000'000);
    (void)H::clock_for(1'000'000);
    (void)H::sck_hz(SpiClock::div4);
    (void)H::max_sck_hz();
    (void)H::ceiling_clock();
    (void)H::reference_hz();
    (void)H::open_drain();
    (void)H::high_speed_read();
    (void)H::write_ahead_from(SpiDataSize::bits16);
    H::prime(SpiMode::mode2, SpiClock::div8, SpiDataSize::bits16);
    (void)H::bit_order(true);
    (void)H::lsb_first();
    typename H::Request r{};
    r.cs = Pin<'A', 4>::ref();
    r.cmd = Borrowed<const uint8_t, Lease::reply>{buf};
    r.cmd_len = 1;
    r.tx = Borrowed<const uint8_t, Lease::reply>{buf};
    r.rx = Borrowed<uint8_t, Lease::reply>{buf + 16};
    r.len = 8;
    r.clock = SpiClock::div2;
    r.mode = SpiMode::mode1;
    r.bits = SpiDataSize::bits16;
    r.polled = true;
    (void)H::start(r);
    r.polled = false;
    (void)H::start(r);
    (void)H::isr();
    (void)H::dma_isr();
    (void)H::status();
    (void)H::recover();
    H::template claim_nss_pad<SpiNss::hardware_input>(true);
    H::template claim_nss_pad<SpiNss::hardware_output>(false);
    H::release();
}

using Client = SpiClient<1>;

void client_verbs() {
    constexpr SysClock clock;
    static uint8_t out[8];
    static uint8_t in[8];
    static uint16_t out16[4];
    static uint16_t in16[4];
    (void)Client::init(clock);
    (void)Client::init(clock, {.mode = SpiMode::mode3, .bits = SpiDataSize::bits16, .lsb_first = true,
                               .nss = SpiNss::software, .drive_output = false});
    static_assert(Client::config_served({.drive_output = false}));
    static_assert(Client::frames_ahead == 1);
    Client::drive_output(true);
    Client::enable(0x5A);
    Client::enable();
    Client::write(0x11); (void)Client::writable();
    (void)Client::poll();
    (void)Client::selected();
    Client::select(true);
    (void)Client::overrun(); Client::clear_overrun();
    (void)Client::crc_error(); Client::clear_crc_error();
    (void)Client::status();
    (void)Client::answer(std::span<const uint8_t>(out), 1000);
    (void)Client::exchange(std::span<const uint8_t>(out), std::span<uint8_t>(in), 1000);
    (void)Client::receive(std::span<uint8_t>(in), 1000);
    (void)Client::exchange(std::span<const uint16_t>(out16), std::span<uint16_t>(in16), 1000);
    (void)Client::isr();
    Client::rxne_interrupt(false); Client::txe_interrupt(false); Client::error_interrupt(false);
    (void)Client::bits();
    (void)Client::disable();
    Client::release();
}

// ---- the hosts, where this package lets them drive -----------------------------
// A template on the pins, so that a package whose MOSI may not be driven
// never forms the host: the CH32X033F8P6 compiles the client alone.
template <SpiPins pins, bool formed = host_column_ok(pins)>
struct Hosts {
    static void run() {}
    static void spi_irq() { (void)Client::isr(); }
    static void dma_irq() {}
};

template <SpiPins pins>
struct Hosts<pins, true> {
    using Host = SpiHost<1, pins>;
    using Host8 = SpiHost<1, pins, DmaTxEngine<3>, DmaRxEngine<2>>;
    using Host16 = SpiHost<1, pins, DmaTxEngine<3, uint16_t>, DmaRxEngine<2, uint16_t>, 600>;
    // Both placements (util/placement.hpp): the default above, and a plain
    // and an engined host whose per-frame paths run from SRAM.
    using RamHost = SpiHost<1, pins, NoDmaEngine, NoDmaEngine, spi_default_hold_off_cycles, InRam>;
    using RamHost16 = SpiHost<1, pins, DmaTxEngine<3, uint16_t>, DmaRxEngine<2, uint16_t>, 600, InRam>;
    static_assert(!Host::has_engines && Host8::has_engines && Host16::has_engines);
    static_assert(!Host::placed_in_ram && RamHost::placed_in_ram && RamHost16::placed_in_ram);
    // The costs follow the placement, and so do the thresholds drawn from
    // them; the hold-off and the write-ahead do not.
    static_assert(RamHost::pump_frame_cycles < Host::pump_frame_cycles &&
                  RamHost::write_ahead_from(SpiDataSize::bits8) == Host::write_ahead_from(SpiDataSize::bits8));
    static_assert(std::is_trivially_copyable_v<typename Host::Request> && sizeof(typename Host::Request) == 40u);
    static_assert(Host::write_ahead_from(SpiDataSize::bits8) ==
                  spi_write_ahead_from(SpiDataSize::bits8, spi_default_hold_off_cycles));

    using PlainBus = SpiBus<Host, P, 4>;
    using TimedBus = SpiBus<Host16, P, 2, BusPassThrough, 50>;
    using Loop = Tenuto<P, PlainBus>;
    using TimedLoop = Tenuto<P, TimedBus>;

    static void run() {
        static uint8_t buf[32];
        host_verbs<Host>(buf);
        host_verbs<Host8>(buf);
        host_verbs<Host16>(buf);
        host_verbs<RamHost>(buf);
        host_verbs<RamHost16>(buf);
        Loop::init_all();
        Loop::step();
        TimedLoop::init_all();
        TimedLoop::step();
        post<PlainBus>(TransferDone{spi_overrun});
        post<TimedBus>(TransferDone{spi_dma_fault});
    }
    static void spi_irq() {
        if (Host::isr()) {
            post<PlainBus>(TransferDone{Host::status()});
        }
        (void)RamHost::isr();
        (void)RamHost16::isr();
    }
    static void dma_irq() {
        if (Host16::dma_isr()) {
            post<TimedBus>(TransferDone{Host16::status()});
        }
        (void)RamHost16::dma_isr();
    }
};

using AllHosts = Hosts<spi1_default_pins>;

void everything() {
    resource_verbs();
    client_verbs();
    AllHosts::run();
}

// The app's glue: the SPI vector and the two channels, bound both ways the
// hardware prologue can be built. The SPI vector is placed in SRAM as a
// program places one of its own plain functions (`.fast_text`), in the
// translation unit that also emits the InRam hosts' trampolines
// (`.fast_text.brio`): the two names are what lets GCC take both kinds in
// one unit (util/placement.hpp, docs/ch32x035/platform.md, "A handler in
// SRAM").
extern "C" BRIO_CH32_INTERRUPT [[gnu::section(".fast_text")]] void spi1_handler() { AllHosts::spi_irq(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() { AllHosts::dma_irq(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() { AllHosts::dma_irq(); }
