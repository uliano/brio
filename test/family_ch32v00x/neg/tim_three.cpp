// mcu: ch32v006k8
// TIM3 is the streamlined block (Tim3), not a Tim<n>: Tim<3> must be
// REFUSED.
#include "ch32v00x/tim.hpp"

void f() { brio::Tim<3>::init(); }
