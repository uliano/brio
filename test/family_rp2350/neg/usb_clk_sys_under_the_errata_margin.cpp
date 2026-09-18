// REFUSED: the controller brought up on a clk_sys that is not at least
// ten per cent above clk_usb. Erratum RP2350-E12 loses eight SIE_STATUS
// flags and six interrupt sources across the two clock domains below
// that margin, so the edge is 52.8 MHz and not 48; here clk_sys is the
// 12 MHz crystal itself.
#include "rp2350/usb.hpp"

using namespace brio;

constexpr Clock<ClockSource::crystal, 12'000'000UL> slow;

bool bring_up() { return Usb::init(slow); }
