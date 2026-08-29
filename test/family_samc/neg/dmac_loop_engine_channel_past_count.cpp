// mcu: samc21e18a samc21g18a samc21j18a
// The channel number is checked WHERE THE ENGINE IS NAMED, not lazily
// when something first touches it: a streaming engine is spelled out in
// an application's type aliases, so channel 12 - one past the twelve
// this family has, and therefore one descriptor slot past the tables
// registered with BASEADDR/WRBADDR - must be refused on that line. The
// serial engines carry the same static_assert for the same reason.
#include "samc/dmac.hpp"
using namespace brio;

void f() { (void)DmaLoopEngine<DMAC_CH_NUM, uint16_t>::channel; }
