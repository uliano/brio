// mcu: samc21j18a
// The MTB's write pointer wraps within a naturally aligned power-of-two
// region (MASTER.MASK is log2(bytes) - 4), so a 200-byte trace buffer
// does not trace 200 bytes - it traces somewhere else.

#include "samc/postmortem.hpp"

using namespace brio;

void instantiate() {
    (void)MtbPostMortem<200, 4>::buffer_bytes;   // meant to FAIL to compile
}
