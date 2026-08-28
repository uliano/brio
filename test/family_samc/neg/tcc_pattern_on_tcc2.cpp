// mcu: samc21e18a samc21g18a samc21j18a
// TCC2 has no pattern generation unit (TCC2_PG is 0), so PATT/PATTBUF
// have nothing to override and a pattern configuration must be refused.

#include "samc/tcc.hpp"

using namespace brio;

static_assert(tcc_pattern_valid(2, 0x1, 0x1),
              "this assertion is meant to FAIL: TCC2 has no pattern generator");
