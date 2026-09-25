// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A DMA-FED CHANNEL ON THE SOFTWARE TRIGGER. 17.2.2.4 raises the DMA
// request "when a trigger event occurs (Software trigger not included)",
// so a stream with SWTRIG as its pace would never be fed: refused.
#include "ch32vx03/dac.hpp"

void f() {
    brio::Dac::configure<1, brio::DacChannelConfig{.triggered = true,
                                                   .trigger = brio::DacTrigger::software,
                                                   .dma = true}>();
}
