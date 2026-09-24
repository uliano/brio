// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// AN ADDRESS OUTSIDE THE MEDIUM'S ZONE, named as a constant. The medium
// lives in the last 4 KB of the array and refuses anything else at run
// time; contains() is the same question asked at compile time, which is
// how a program pins an address it knows when it is built. Here the
// address is in the image's own flash.
#include "ch32vx03/nvm_flash.hpp"

static_assert(brio::MainFlashPartition::contains(0x0100u, brio::Flash::page_size));

void f() {}
