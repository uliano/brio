// mcu: avr128da48
// The AVR DA48 carries ZCD0 and ZCD1, not ZCD2 (the DB48 carries all
// three): the generator of ZCD2 names an instance that is not there.
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { (void)EvZcdOut<2>::code; }
