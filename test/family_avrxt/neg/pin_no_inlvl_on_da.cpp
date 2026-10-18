// mcu: avr128da48 avr128da28
#include "avrxt/pin.hpp"
using namespace brio;
void f() { Pin<'D', 3>::configure({.input_level = PinLevel::ttl}); }
