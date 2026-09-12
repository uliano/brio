// Must FAIL: a client frames on its select pad; without one it is not a client.
#include "rp2040/spi.hpp"

constexpr brio::SpiPins no_cs{.sck = 10, .tx = 11, .rx = 8};
void up() { (void)brio::SpiClient<1, no_cs>::selected(); }
