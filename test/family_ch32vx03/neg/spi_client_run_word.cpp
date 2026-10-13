// mcu: ch32v203f6 ch32v203c8 ch32v303vc
// A CLIENT'S RUN MOVES FRAMES: DFF gives this block 8-bit frames or
// 16-bit ones (RM 20.4.1), DATAR is a 16-bit register, so a run over
// words has no frame to be - refused at compile time.
#include "ch32vx03/spi.hpp"

#include <span>

void f() {
    static uint32_t out[2] = {1, 2};
    static uint32_t in[2] = {};
    (void)brio::SpiClient<1>::exchange(std::span<const uint32_t>(out), std::span<uint32_t>(in), 100u);
}
