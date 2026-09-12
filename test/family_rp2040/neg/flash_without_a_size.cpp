// Must FAIL: the build states the chip's size; a TU that undefines it
// has no flash to speak of.
#undef BRIO_RP2040_FLASH_KB
#include "rp2040/flash.hpp"
