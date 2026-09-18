// OTP is read-only in this tree: no verb programs a row.
#include "rp2350/otp.hpp"
void f() { (void)brio::Otp::program(0u, 0x1234u); }
