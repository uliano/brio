// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// DFF IS ONE BIT: 8-bit frames or 16-bit ones and nothing else (RM
// 20.4.1). A third width cast into the field would spill into CRCNEXT
// beside it, so a configuration that names one is refused where it is a
// constant.
#include "ch32v203/spi.hpp"

void f() {
    brio::Spi<1>::configure<brio::SpiConfig{.bits = static_cast<brio::SpiDataSize>(2)}>();
}
