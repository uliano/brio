// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE DUAL-EDGE CAPTURE of the CH32V30x_D8 (RM 14.3.11, 15.3.9, 14.4.21,
// 15.4.25): TIMx_AUX's three bits put channels 2..4 of a timer into a
// capture of BOTH edges whose register holds the pulse width - every
// CH32V303 has it on every timer with channels, and no CH32V203 (the
// verb answers false there: tim.cpp). Channel 1 has no bit, and a basic
// timer no channel at all.
#include "ch32vx03/tim.hpp"

using namespace brio;

static_assert(Tim<1>::has_dual_edge_capture && Tim<2>::has_dual_edge_capture &&
              Tim<4>::has_dual_edge_capture);
static_assert(tim_cap_ed_ch2 == 1u && tim_cap_ed_mask == 7u);
static_assert(offsetof(TimRegs, AUX) == 0x50);

void exercise_dual_edge() {
    using T = Tim<2>;
    T::init();
    (void)T::configure({.prescaler = 143, .period = 0xFFFF});
    (void)T::dual_edge_capture(0);                       // false: channel 1 has no bit
    (void)T::dual_edge_capture(1, TimCapturePolarity::rising, 3, TimCapturePrescaler::every);
    (void)T::dual_edge_capture(3, TimCapturePolarity::falling);
    (void)T::dual_edge(1);
    (void)T::compare(1);
    (void)T::dual_edge_capture_off(1);
    (void)T::dual_edge_capture_off(0);                   // false
    T::release();

    (void)Tim<1>::dual_edge_capture(2);
    (void)Tim<3>::dual_edge_capture(1);
}
