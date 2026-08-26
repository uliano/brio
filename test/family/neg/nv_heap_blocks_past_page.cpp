// mcu: avr128db48 avr128da28
// A map version is ONE erase-unit-sized write, so the block table has a
// physical ceiling: 512 bytes hold a header, a checksum and 35 entries
// of 14 bytes. Asking for 64 blocks on this media is not a heap that
// wastes space, it is a heap whose map cannot be written at all - and
// the failure has to happen here, not at the first seal on the bench.
#include "avrdx/nvm_flash.hpp"
#include "util/nv_heap.hpp"
using namespace brio;
NvHeap<NvmFlash, 64> heap;
