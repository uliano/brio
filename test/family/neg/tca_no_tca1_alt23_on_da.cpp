// mcu: avr128da64
// DS80000882C 2.11.1: TCA1's ALT2 (PORTE) and ALT3 (PORTG) do not work on
// any DA revision - the routes must be refused there.
#include "avrdx/tca.hpp"
using namespace brio;
void f() { Tca<1>::init<TcaConfig{.mode = TcaMode::normal, .period = 1, .route = 'G'}>(); }
