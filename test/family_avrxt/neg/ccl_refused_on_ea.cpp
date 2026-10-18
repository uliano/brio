// mcu: avr64ea48 avr64ea28
// On the AVR EA input 2 selects AC1 under the AC code, which LutInput::ac
// does not spell: ccl.hpp refuses a part without AC2 by name.
#include "avrxt/ccl.hpp"
using namespace brio;
void f() {}
