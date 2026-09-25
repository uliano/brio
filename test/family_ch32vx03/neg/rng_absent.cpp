// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb
// THE RANDOM NUMBER GENERATOR ON A PART THAT HAS NONE. The CH32V303
// datasheet's table 2-1-1 gives the RNG to the CH32V303RC and VC and a
// dash to the CB and the RB - the same device class - and the CH32V203's
// table 2-1 has no RNG row at all. The header compiles everywhere; the
// type is refused where it is named.
#include "ch32vx03/rng.hpp"

void f() { (void)brio::Rng::read(); }
