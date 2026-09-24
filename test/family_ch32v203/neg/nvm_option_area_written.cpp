// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A WRITE INTO THE OPTION BYTE AREA. This driver reads the option bytes
// and writes none of them - releasing read protection erases the whole
// chip (RM 32.6.4) and the rest is provisioning - so the only view it
// hands out is const, and a store through it does not compile.
#include "ch32v203/nvm.hpp"

void f() { *brio::FlashOptionArea::raw() = 0xFFFFA55Au; }
