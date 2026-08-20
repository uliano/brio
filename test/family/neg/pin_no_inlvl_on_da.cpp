// mcu: avr128da48 avr128da28
#include "avrdx/pin.hpp"
using namespace brio;
void f() { Pin<'D', 3>::configure({.input_level = PinLevel::ttl}); }
