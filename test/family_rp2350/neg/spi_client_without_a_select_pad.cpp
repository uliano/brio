// A PL022 client frames on SSPFSSIN, so its select IS a pad: a client
// pin set with no CSn must not compile.
#include "rp2350/spi.hpp"
constexpr brio::SpiPins pins{.sck = 10, .tx = 11, .rx = 8};
using Bad = brio::SpiClient<1, pins>;
void f() { (void)sizeof(Bad); }
