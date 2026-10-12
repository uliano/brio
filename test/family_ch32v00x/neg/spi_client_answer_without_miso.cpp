// mcu: ch32v006k8 ch32v003f4
// A client whose pins name no MISO pad has nothing to answer on: an
// answering run on it is refused (a receive run is not).
#include "ch32v00x/spi.hpp"

using Listener = brio::SpiClient<1, brio::SpiPins{.sck = {'C', 5}, .mosi = {'C', 6}, .nss = {'C', 1}}>;
uint8_t out[4];
void f() { (void)Listener::answer(std::span<const uint8_t>(out), 100u); }
