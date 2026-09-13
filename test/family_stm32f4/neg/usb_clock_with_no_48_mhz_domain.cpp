// The controller needs 48 MHz exactly on its own domain, which on this
// family is the main PLL's Q output: 180 MHz gives 45 MHz there and is
// refused, whatever the ladder says about the core rate.
// mcu: stm32f429xx stm32f446xx
#include "stm32f4/usb.hpp"
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
void f() { (void)brio::UsbFs::init(SysClock{}); }
