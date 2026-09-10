// SPI family smoke TU: ch32v00x/spi.hpp's resource, host and client
// instantiated, the host in both engine arrangements and with the
// SpiBus arbiter of util/spi_bus.hpp over it - instantiation only, no
// main(), no hardware. The chapter's arithmetic is pinned in the header
// itself; what this fixture adds is the util contract: SpiHost is the
// Bus a BusMaster accepts, on this architecture as on the other three.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/spi.hpp"
#include "kernel/kernel.hpp"
#include "util/spi_bus.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

static_assert(spi_base_for(1) == 0x40013000 && spi_base_for(2) == 0);
static_assert(Spi<1>::dma_rx_channel == 2 && Spi<1>::dma_tx_channel == 3);
static_assert(offsetof(SpiRegs, HSCR) == 0x24);

using Host = SpiHost<1>;
using HostDma = SpiHost<1, spi1_default_pins, DmaTxEngine<3>, DmaRxEngine<2>>;
constexpr SpiPins write_only{.sck = {'C', 5}, .mosi = {'C', 6}};
using HostNoMiso = SpiHost<1, write_only>;
using Client = SpiClient<1>;

static_assert(!Host::has_engines && HostDma::has_engines);
static_assert(std::is_trivially_copyable_v<Host::Request>);
static_assert(std::is_same_v<Host::Request, Host::Request>);

using PlainBus = SpiBus<Host, P, 4>;
using DmaBus = SpiBus<HostDma, P, 2, BusPassThrough, 100>;

void resource_verbs() {
    using S = Spi<1>;
    S::bus_clock(true);
    S::reset();
    (void)S::configure({.role = SpiRole::host, .mode = SpiMode::mode3, .clock = SpiClock::div2,
                        .bits = SpiDataSize::bits16, .nss = SpiNss::hardware_output, .crc = true});
    S::enable();
    (void)S::enabled();
    S::data(SpiDataSize::bits16, 0x1234);
    (void)S::data(SpiDataSize::bits8);
    S::data8(0x55);
    (void)S::data8();
    (void)S::rxne(); (void)S::txe(); (void)S::busy(); (void)S::status();
    S::flush_rx();
    (void)S::overrun(); S::clear_overrun();
    (void)S::mode_fault(); S::clear_mode_fault();
    (void)S::crc_error(); S::clear_crc_error();
    S::crc_next(); (void)S::rx_crc(); (void)S::tx_crc();
    S::software_select(true);
    S::high_speed_read(true);
    S::dma_requests(true, false);
    S::rxne_interrupt(true); S::txe_interrupt(false); S::error_interrupt(true);
    (void)S::isr();
    (void)S::disable();
}

template <typename H>
void host_verbs(uint8_t* buf) {
    constexpr SysClock clock;
    (void)H::init(clock, 8'000'000);
    H::rebase(24'000'000);
    (void)H::clock_for(1'000'000);
    (void)H::sck_hz(SpiClock::div4);
    (void)H::max_sck_hz(); (void)H::ceiling_clock(); (void)H::reference_hz();
    H::prime(SpiMode::mode0, SpiClock::div8, SpiDataSize::bits16);
    (void)H::bit_order(true); (void)H::lsb_first();
    typename H::Request r{};
    r.cs = Pin<'C', 1>::ref();
    r.cmd = Borrowed<const uint8_t, Lease::reply>{buf};
    r.cmd_len = 1;
    r.tx = Borrowed<const uint8_t, Lease::reply>{buf};
    r.rx = Borrowed<uint8_t, Lease::reply>{buf + 8};
    r.len = 4;
    (void)H::start(r);
    r.polled = true;
    (void)H::start(r);
    (void)H::isr();
    (void)H::dma_isr();
    (void)H::status();
    (void)H::recover();
    H::claim_nss_pad(true);
    H::release();
}

void all_hosts() {
    static uint8_t buf[16];
    host_verbs<Host>(buf);
    host_verbs<HostDma>(buf);
    host_verbs<HostNoMiso>(buf);
}

void client_verbs() {
    constexpr SysClock clock;
    (void)Client::init(clock, {.mode = SpiMode::mode1, .nss = SpiNss::hardware_input, .drive_output = false});
    Client::drive_output(true);
    Client::enable(0xA5);
    Client::write(0x5A);
    (void)Client::writable();
    (void)Client::poll();
    (void)Client::selected();
    Client::select(true);
    (void)Client::overrun(); Client::clear_overrun();
    (void)Client::crc_error(); Client::clear_crc_error();
    (void)Client::status();
    (void)Client::isr();
    Client::rxne_interrupt(true); Client::txe_interrupt(true); Client::error_interrupt(true);
    (void)Client::bits();
    static_assert(Client::frames_ahead == 1);
    (void)Client::disable();
    Client::release();
}

// The arbiter over both hosts: the kernel accepts them as active
// objects, which is the util contract's whole claim.
using Loop = Kernel<P, PlainBus>;
using DmaLoop = Kernel<P, DmaBus>;

void arbiters() {
    Loop::init_all();
    Loop::step();
    DmaLoop::init_all();
    DmaLoop::step();
    post<PlainBus>(TransferDone{spi_ok});
    post<DmaBus>(TransferDone{spi_dma_fault});
}
