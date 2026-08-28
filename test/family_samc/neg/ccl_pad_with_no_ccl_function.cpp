// mcu: samc21e18a samc21g18a samc21j18a
// PA12 carries SERCOM2/PAD0, TCC2/WO0, TCC0/WO6 and AC/CMP0 - and no
// CCL function on any package of this family. The reserve's pad map
// says so and the pad type refuses rather than muxing function I onto
// a pad where it means nothing.

#include "samc/ccl.hpp"

using namespace brio;

void use() { CclIn<Pin<'A', 12>>::claim(); }
