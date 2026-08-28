// mcu: samc21e18a samc21g18a samc21j18a
// This family has two converters. A third is not a runtime error but a
// type that must not exist.

#include "samc/adc.hpp"

using namespace brio;

void use() { (void)Adc<2>::enabled(); }
