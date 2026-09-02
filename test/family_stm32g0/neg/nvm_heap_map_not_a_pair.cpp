// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The heap's map is a PING-PONG: with one map page a mutation would erase
// the only copy and the heap would not be crash-safe. Two is the minimum
// (util/nv_heap.hpp), and on this target one page is 2 Kbytes of the
// storage bank - cheap enough that there is no reason to try.
#include "stm32g0/nvm_flash.hpp"
#include "util/nv_heap.hpp"
using namespace brio;
NvHeap<MainFlash, 8, 1> heap;
void f() { (void)heap.mount(); }
