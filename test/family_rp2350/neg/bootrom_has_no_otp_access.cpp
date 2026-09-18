// The bootrom's otp_access carries an IS_WRITE flag, so it is not
// wrapped here at all.
#include "rp2350/bootrom.hpp"
void f() {
    uint8_t buf[2] = {};
    (void)brio::Bootrom::otp_access(buf, 2u, 0u);
}
