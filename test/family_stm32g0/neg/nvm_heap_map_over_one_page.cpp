// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A map version is ONE erase-unit-sized write (util/nv_heap.hpp), so
// max_blocks is bounded by what a page holds: 16 header bytes + 14 per
// entry + 2 of CRC must fit in 2048. 200 blocks want 2818 bytes and are
// refused - the alternative would be a map version torn across two
// erases, which is exactly the atomicity the ping-pong exists to give.
#include "stm32g0/nvm_flash.hpp"
#include "util/nv_heap.hpp"
using namespace brio;
NvHeap<MainFlash, 200, 2> heap;
void f() { (void)heap.mount(); }
