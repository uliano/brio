// NLR.PL is fourteen bits (11.5.18), so a rectangle of 16384 pixels a
// line does not fit the register and is refused before the store rather
// than truncated into a narrower one.
// mcu: stm32f429xx stm32f427xx stm32f469xx stm32f446xx
#include "stm32f4/dma2d.hpp"
static_assert(brio::dma2d_area_valid(brio::Dma2dArea{.pixels = 16384, .lines = 1}),
              "PL holds fourteen bits");
