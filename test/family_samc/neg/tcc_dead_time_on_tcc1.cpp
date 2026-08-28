// mcu: samc21e18a samc21g18a samc21j18a
// TCC1 has no dead-time insertion unit (TCC1_DTI is 0 in the device
// header), so the complementary-pair task must refuse it. This is the
// per-instance extension refusal the whole reserve exists for: the
// register bits are there in the memory map, and writing them would be
// silent.

#include "samc/tcc.hpp"

using namespace brio;

using Pair = TccPairPwm<Tcc<1>, 0, 999>;
static_assert(Pair::max == 999,
              "this assertion is meant to FAIL: TCC1 has no dead-time unit");
