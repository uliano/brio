// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE SAME REFUSAL THROUGH THE MEDIUM. The FlashMedia contract's
// erase() takes an address and nothing else, so the rate arrives with
// the TYPE: a medium written with a clock that needs HCLK halved around
// a write (RM 32.1) does not compile at all.
#include "ch32vx03/nvm_flash.hpp"

using Full = brio::Clock<brio::ClockSource::pll, 144'000'000>;
using Store = brio::MainFlash<Full>;

void f() { (void)Store::erase(brio::MainFlashPartition::storage_base); }
