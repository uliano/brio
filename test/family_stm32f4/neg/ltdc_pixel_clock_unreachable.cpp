// The pixel clock's PLL has NO input divider of its own: it divides the
// main PLL's M (6.3.24), so a root and an M that put the VCO input
// outside its 1..2 MHz window make no pixel clock at all - here a 25 MHz
// crystal with the divider an 8 MHz one wants.
// mcu: stm32f429xx stm32f469xx stm32f446xx stm32f411xe
#include "stm32f4/ltdc.hpp"
static_assert(brio::lcd_clock_config_for(25'000'000u, 6'000'000u, 4).pll.n != 0,
              "no exact PLLSAI triple for this pixel clock");
