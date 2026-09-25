// mcu: ch32x035f8 ch32x033f8
// USART family smoke TU, the 20-pin parts' own columns: the CH32X035F8U6
// brings USART4 out on PB0/PB1 and on the USB pads PC16/PC17 (the one
// package where those two are not shorted to PC11/PC10) and USART3 on the
// debug pads; the CH32X033F8P6 brings USART1 out on PA10/PA11 (its code 1)
// and USART4 on PA5/PA9 (its code 1), its default TX PB0 sharing a pin
// with PA7.
#include "ch32x035/clock.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

using namespace brio;

using P = Ch32x035Platform<>;
using SysClock = Clock<ClockSource::internal, 48'000'000>;

template <uint8_t n, uint8_t code>
bool open_where_valid() {
    if constexpr (device::has_usart(n) && usart_remap_valid(n, code)) {
        using S = Uart<n, P, 16, 16, UartFormat{}, NoDmaEngine, NoDmaEngine, code>;
        constexpr SysClock clock;
        const bool ok = S::init(clock, 115200);
        S::release();
        return ok;
    } else {
        return false;
    }
}

void columns() {
    // The CH32X035F8U6's three.
    (void)open_where_valid<4, 0>();   // PB0/PB1
    (void)open_where_valid<4, 2>();   // PC16/PC17
    (void)open_where_valid<3, 1>();   // PC18/PC19: false while the probe owns them
    // The CH32X033F8P6's.
    (void)open_where_valid<1, 1>();   // PA10/PA11
    (void)open_where_valid<4, 1>();   // PA5/PA9
}

static_assert(device::package != Package::qfn20 || usart_remap_valid(4, 2));
static_assert(device::package != Package::qfn20 || !device::has_usart(1));
static_assert(device::package != Package::tssop20 || !usart_remap_valid(4, 0));   // PB0 shares PA7's pin
static_assert(device::package != Package::tssop20 || usart_remap_valid(4, 1));
