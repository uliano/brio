// mcu: ch32v203g6 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// SPI1's SECOND COLUMN (table 10-32, AFIO_PCFR1's SPI1 bit): the one
// remap this family's SPI has, on the parts whose package brings out
// every pad of it - PA15, PB3, PB4, PB5. The other three parts of the
// series bond none of PB3/PB4 and no PA15, so the column is not a
// disconnection there but an absence, and a host named on it is refused
// at compile time (neg/spi_remap_pads_absent.cpp is that refusal).
//
// SPI2 has no remap field at all, so there is no second column to test
// for it: one column, from the datasheet's pin table.
#include "ch32v203/clock.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/spi.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 144'000'000>;

inline constexpr SpiPins moved = spi_pins_for(1, 1);

static_assert(moved.nss == Pad{'A', 15} && moved.sck == Pad{'B', 3} &&
              moved.miso == Pad{'B', 4} && moved.mosi == Pad{'B', 5});
static_assert(moved.remap == 1u, "the column travels with its pads");
static_assert(spi_pins_valid(1, moved), "this package bonds the whole column");
static_assert(afio_remap_has_code(Remap::spi1, 1), "and afio.hpp agrees");

using Moved = SpiHost<1, moved>;
using MovedClient = SpiClient<1, moved>;

static_assert(Moved::pin_pads.sck == Pad{'B', 3});
static_assert(MovedClient::has_nss_pad);

void use()
{
    constexpr SysClock clock{};
    (void)Moved::init(clock);
    Moved::claim_nss_pad(true);
    Moved::release();
    (void)MovedClient::init(clock);
    MovedClient::release();
}

int main()
{
    use();
    return 0;
}
