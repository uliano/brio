// mcu: samc21e18a
// The E bonds no PORT B pad to the CCL at all: PB08 is CCL2/IN[8] on
// the G and the J and nothing on the E.

#include "samc/ccl.hpp"

using namespace brio;

void use() { CclIn<Pin<'B', 8>>::claim(); }
