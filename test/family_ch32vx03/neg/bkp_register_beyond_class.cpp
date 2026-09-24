// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8
// A BACKUP DATA REGISTER THIS DEVICE CLASS HAS NOT GOT. RM 4.3's note
// under table 4-1 gives BKP_DATAR11..42 to the CH32V20x_D8 and its
// relatives; the CH32V20x_D6 carries ten, so the eleventh is refused
// where the index is a constant - and the part whose class does carry
// it is left off the list above, because there the same line compiles.
#include "ch32vx03/rtc.hpp"

void f() { (void)brio::Bkp::data<11>(); }
