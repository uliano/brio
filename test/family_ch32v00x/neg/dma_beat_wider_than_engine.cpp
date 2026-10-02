// mcu: ch32v006k8 ch32v003f4
// An engine's Elem is the WIDEST beat its binding moves: a byte engine
// (a USART's DATAR) handed a run of half-words must be REFUSED.
#include <span>

#include "ch32v00x/dma.hpp"

void f(const uint16_t* run) { (void)brio::DmaTxEngine<4>::start(std::span<const uint16_t>(run, 8)); }
