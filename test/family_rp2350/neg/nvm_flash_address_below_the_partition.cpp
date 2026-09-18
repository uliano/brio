// The storage partition is the top 64 kB of the chip: an address in the
// image's own flash is not in it, and the compile-time half of the
// bounds check is what says so before a build ever runs.
#include "rp2350/nvm_flash.hpp"
static_assert(brio::QspiFlashPartition::contains(0x0000'1000u, 256u),
              "an address below the partition floor is not storage");
void f() {}
