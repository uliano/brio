// Must FAIL: the instruction memory holds thirty-two instructions.
#include "rp2040/pio.hpp"

brio::PioProgram<33> too_long{};
