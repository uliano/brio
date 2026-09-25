// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303rc ch32v303vc
// THE MEDIUM'S ZONE OUTSIDE THE WINDOW. The FlashMedia's zone is the
// top 4 KB of the zero-wait window and nothing above it - a placement
// and not a reach, the engine's two methods writing the tail too
// (nvm_flash.hpp's header gives the three reasons): a page of the tail
// is not in the zone, and contains() - the compile-time half of the
// medium's own bounds - says so where the address is a constant.
#include "ch32vx03/nvm_flash.hpp"

static_assert(brio::MainFlashPartition::contains(brio::Flash::tail_base, brio::Flash::page_size));

void f() {}
