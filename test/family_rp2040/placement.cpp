// Placement family smoke TU: util/placement.hpp on this stratum - the
// platform says places_fast_text, and the three IP drivers' hosts take
// InRam as well as InFlash: SpiHost (start, the pump, the DMA line),
// Uart (the vector, the DMA line), I2cHost (the vector, the DMA line).
// THE COMDAT TRAP, proved absent: the InRam instances (.fast_text.brio)
// and the flash engine's window (.ram_text.brio), both a header's grouped
// functions, are emitted in the same translation unit as a program's own
// plain function in .fast_text and another in .ram_text - which GCC
// refuses when a grouped and a plain function share one section name.
#include "rp2040/dma.hpp"
#include "rp2040/flash.hpp"
#include "rp2040/nvm_flash.hpp"
#include "rp2040/i2c.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/spi.hpp"
#include "rp2040/uart.hpp"
#include "util/placement.hpp"

using namespace brio;

static_assert(platform_places_fast_text<Rp2040Platform<>> && platform_places_fast_text<Rp2040Platform<1>>);
static_assert(HotPlacement<InRam, Rp2040Platform<>> && HotPlacement<InFlash, Rp2040Platform<>>);

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SpiPins spi_pins{.sck = 18, .tx = 19, .rx = 16};
using RamHost = SpiHost<0, spi_pins, NoDmaEngine, NoDmaEngine, InRam>;
using RamDmaHost = SpiHost<0, spi_pins, DmaTxEngine<4, uint16_t>, DmaRxEngine<5, uint16_t>, InRam>;
constexpr UartPins uart_pins{.tx = {4, PinFunction::uart}, .rx = {5, PinFunction::uart}};
using RamUart = Uart<1, uart_pins, 64, 256, NoDmaEngine, NoDmaEngine, InRam>;
using RamEngined = Uart<1, uart_pins, 64, 256, DmaTxEngine<2>, DmaRxEngine<3>, InRam>;
constexpr I2cPins i2c_pins{.scl = 3, .sda = 2};
using RamI2c = I2cHost<1, i2c_pins, NoDmaEngine, NoDmaEngine, InRam>;
using RamDmaI2c = I2cHost<1, i2c_pins, DmaTxEngine<6, uint16_t>, DmaRxEngine<7>, InRam>;

uint8_t buf[16];
uint8_t page[Flash::page_size];

/// A program's own placed functions, plain, beside the grouped ones.
extern "C" [[gnu::section(".fast_text")]] void placed_vector() {
    (void)RamHost::isr();
    (void)RamDmaHost::dma_isr();
    (void)RamUart::isr();
    (void)RamEngined::dma_isr();
    (void)RamI2c::isr();
    (void)RamDmaI2c::dma_isr();
}
extern "C" [[gnu::section(".ram_text")]] void must_run_from_sram() { buf[0] = 1; }

void placement_verbs() {
    constexpr SysClock clock;
    (void)RamHost::init(clock);
    RamHost::Request r{};
    r.cs = Pin<17>::ref();
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    r.len = 16;
    (void)RamHost::start(r);
    r.polled = true;
    (void)RamHost::start(r);
    (void)RamHost::isr();
    (void)RamDmaHost::init(clock);
    RamDmaHost::Request d{};
    d.cs = Pin<17>::ref();
    d.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    d.len = 16;
    (void)RamDmaHost::start(d);
    (void)RamDmaHost::isr();
    (void)RamDmaHost::dma_isr();
    (void)RamUart::init(clock, 115200);
    (void)RamUart::isr();
    (void)RamEngined::init(clock, 115200);
    (void)RamEngined::isr();
    (void)RamEngined::dma_isr();
    (void)RamI2c::init(clock);
    (void)RamI2c::isr();
    (void)RamDmaI2c::init(clock);
    (void)RamDmaI2c::isr();
    (void)RamDmaI2c::dma_isr();
    (void)Flash::erase(QspiFlashPartition::storage_base, Flash::sector_size);
    (void)Flash::program(QspiFlashPartition::storage_base, page);
}
