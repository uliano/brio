// The instruction memory holds thirty-two instructions, so a program of
// thirty-three could never be loaded whole.
#include "rp2350/pio.hpp"
void f() {
    constexpr brio::PioProgram<33> p{};
    (void)p.valid();
}
