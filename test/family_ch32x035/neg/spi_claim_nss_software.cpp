// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7
// A software select (SSM) leaves the NSS pad free (16.2.1): there is no
// pad to claim for it, and the claim is refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/spi.hpp"

void f() {
    brio::SpiHost<1>::claim_nss_pad<brio::SpiNss::software>(true);
}
