// mcu: samc21j18a
// CTRLB.SLEEPPRM code 0x2 is Reserved (27.8.2). The enum does not name
// it, so only a cast reaches it - and the config check must still refuse.

#include "samc/nvm.hpp"

using namespace brio;

void bad() {
    constexpr NvmConfig cfg{.sleep_power = static_cast<NvmSleepPower>(0x2)};
    Nvm::init<cfg>();
}
