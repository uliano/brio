// Must FAIL: SPI0's clock is not on GPIO 10 (that is SPI1's), table 279.
#include "rp2040/spi.hpp"

constexpr brio::SpiPins wrong{.sck = 10, .tx = 19, .rx = 16};
void up() { (void)brio::SpiHost<0, wrong>::status(); }
