// mcu: samc21j18a
// A post-mortem store that keeps no packets: the record would be a magic
// word and a checksum over nothing, and the boot side would learn only
// that something had died - which the PanicRecord already says.

#include "samc/postmortem.hpp"

using namespace brio;

void instantiate() {
    (void)MtbPostMortem<256, 0>::buffer_bytes;   // meant to FAIL to compile
}
