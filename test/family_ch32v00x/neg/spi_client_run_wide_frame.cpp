// mcu: ch32v006k8 ch32v003f4
// DFF gives this block frames of eight bits or sixteen (RM 16.3.1): a
// client run over a wider element has no frame to carry it and is
// refused.
#include "ch32v00x/spi.hpp"

using Client = brio::SpiClient<1>;
uint32_t out[4];
uint32_t in[4];
void f() { (void)Client::exchange(std::span<const uint32_t>(out), std::span<uint32_t>(in), 100u); }
