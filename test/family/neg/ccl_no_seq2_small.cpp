// mcu: avr128db28 avr128da28
#include "avrdx/ccl.hpp"
using namespace brio;
void f() { Ccl::sequencer<2>(Sequencer::jk_flip_flop); }
