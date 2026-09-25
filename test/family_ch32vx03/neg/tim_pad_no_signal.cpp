// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A PAD THAT CARRIES NO SIGNAL IS NOT A PAD. TIM5's external trigger
// reaches no pin of either series (the datasheets' pin tables name no
// TIM5_ETR, and TIM5 has no remap column at all), so its entry is an
// invalid Pad - on the parts that have the timer and on those that have
// not - and TimPad refuses it.
#include "ch32vx03/tim.hpp"

using NoEtr = brio::TimPad<brio::tim_etr_pad(5, 0)>;
void f() { NoEtr::claim(); }
