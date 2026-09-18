// Must FAIL: a chip traits type whose pads cannot be driven low by hand
// is not a DwApbI2cChip - the unstick owns the wire for nine pulses and
// a STOP, and the concept says which verb is missing.
#include "dw_apb_i2c/i2c.hpp"
#include "host/sim_dw_apb_i2c.hpp"

namespace {
struct NoOpenDrain : brio::SimDwApbI2c {
    template <uint8_t pin>
    struct Pad : brio::SimDwApbI2cPad<pin> {
        static void drive_low() = delete;
    };
};
}   // namespace

void f() { (void)brio::DwApbI2cBlock<NoOpenDrain, 0>::enabled(); }
