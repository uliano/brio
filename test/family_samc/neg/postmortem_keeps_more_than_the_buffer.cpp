// mcu: samc21j18a
// Keeping more packets than the rolling buffer can hold: a 256-byte
// buffer is 32 packets, so the last 33 do not exist and the extra slot
// of the record could never be filled.

#include "samc/postmortem.hpp"

using namespace brio;

void instantiate() {
    (void)MtbPostMortem<256, 33>::buffer_bytes;   // meant to FAIL to compile
}
