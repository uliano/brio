// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A PAD THAT CARRIES NO SIGNAL IS NOT A PAD. TIM4's external trigger
// reaches no pin of this series (table 10-18 names none), so its entry
// in the column is an invalid Pad and TimPad refuses it.
#include "ch32v203/tim.hpp"

using NoEtr = brio::TimPad<brio::tim_etr_pad(4, 0)>;
void f() { NoEtr::claim(); }
