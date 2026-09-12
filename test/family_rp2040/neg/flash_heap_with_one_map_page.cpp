// Must FAIL: the heap's map pair wants two erase units to ping-pong.
#include "rp2040/nvm_flash.hpp"

brio::NvHeap<brio::QspiFlash, 8, 1> heap;
