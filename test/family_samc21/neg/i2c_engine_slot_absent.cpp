// mcu: samc21e18a samc21g18a samc21j18a
// The I2C host has no engine slot: every tenure runs on the smart-mode
// pump, the DMAC being driven for the Uart's transmitter alone
// (samc21/dmac.hpp, erratum 1.10.4). An engine argument fails to compile.
#include "samc21/i2c.hpp"
using namespace brio;

struct SomeEngine {
    static constexpr bool present = true;
    static constexpr uint8_t channel = 6;
};

constexpr I2cPads pads{
    .sda_pin = {'A', 8, PinFunction::c},
    .scl_pin = {'A', 9, PinFunction::c},
};

void f() { (void)I2cHost<0, pads, 0, SomeEngine, SomeEngine>::status(); }
