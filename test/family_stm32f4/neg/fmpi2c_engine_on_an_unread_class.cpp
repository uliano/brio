// No device header of this pack carries a request mapping, so the reserve
// keys this block's slice on the part class - and the F410, F412 and
// F413/F423 reference manuals are not on this desk. An engine there is
// REFUSED: these are the same two cells that are correct on the F446.
// mcu: stm32f412zx stm32f413xx stm32f410rx
#include "stm32f4/dma.hpp"
#include "stm32f4/fmpi2c.hpp"
using namespace brio;
constexpr FmpI2cPins pins{.scl = {'B', 10, PinFunction::af4},
                          .sda = {'B', 11, PinFunction::af4}};
using Bus = FmpI2cHost<1, pins, DmaTxEngine<1, 5, 2>, DmaRxEngine<1, 2, 2>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
