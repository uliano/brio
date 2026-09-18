// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A RUN THAT IS NOT WHOLE CELLS. The cell of this medium IS the page -
// 256 bytes, the only grain the fast program writes (RM 32.5.6) - so a
// run of sixty-four bytes is not something program() can put down, and
// cell_aligned() is where a constant says so at compile time.
#include "ch32v203/nvm_flash.hpp"

static_assert(brio::MainFlashPartition::cell_aligned(brio::MainFlashPartition::storage_base, 64u));

void f() {}
