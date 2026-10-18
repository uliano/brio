// mcu: avr128db28 avr128da28 avr64ea28 avr64ea48
#include "avrxt/ccl.hpp"
using namespace brio;
void f() { Ccl::sequencer<2>(Sequencer::jk_flip_flop); }
