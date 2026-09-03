// mcu: stm32g031xx
// The G031 class has no comparator at all: its header declares neither
// COMP1_BASE nor COMP2_BASE, so even the first instance is a refusal.
#include "stm32g0/comp.hpp"
brio::Comp<1> c;
