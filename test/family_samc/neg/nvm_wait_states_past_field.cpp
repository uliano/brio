// mcu: samc21j18a
// CTRLB.RWS is a FOUR-BIT field (bits 4:1, header Pos = 1): a sixteenth
// wait state does not exist and would mask away to zero, leaving the
// flash too slow for the clock that asked for it. Must not compile.

#include "samc/nvm.hpp"

using namespace brio;

void bad() {
    constexpr NvmConfig cfg{.wait_states = 16};
    Nvm::init<cfg>();
}
