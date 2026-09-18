// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A PVD THRESHOLD PAST ITS FIELD. PLS is three bits and the chapter
// gives eight levels (RM 2.4.1), so a ninth is not a threshold this
// silicon has - refused where the level is a constant, where the
// run-time form can only answer false.
#include "ch32v203/pwr.hpp"

void f() {
    (void)brio::Pwr::pvd<static_cast<brio::PvdLevel>(8)>(true);
}
