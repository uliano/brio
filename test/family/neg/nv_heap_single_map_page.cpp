// mcu: avr128db48 avr128da28
// The map is a PING-PONG: a new version is written into a page that is
// erased first, and the previous version rules until it lands. With one
// page in the rotation that erase would take the only copy of the map
// down, and a power loss between the erase and the write would empty the
// heap. One page is not a smaller heap, it is a heap without the
// property the whole design exists for.
#include "avrdx/nvm_flash.hpp"
#include "util/nv_heap.hpp"
using namespace brio;
NvHeap<NvmFlash, 8, 1> heap;
