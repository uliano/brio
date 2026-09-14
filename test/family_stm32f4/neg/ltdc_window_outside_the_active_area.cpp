// A layer's window must lie inside the ACTIVE display area (16.4.2): the
// registers would hold a position past the last visible pixel and fetch
// a frame buffer nobody sees.
// mcu: stm32f429xx stm32f469xx stm32f446xx stm32f411xe
#include "stm32f4/ltdc.hpp"
using namespace brio;
constexpr LtdcTiming qvga{.hsync = 10, .hbp = 20, .width = 240, .hfp = 10,
                          .vsync = 2, .vbp = 2, .height = 320, .vfp = 4};
constexpr LtdcLayerConfig off_the_edge{
    .window = {1, 0, 240, 320}, .format = LtdcPixelFormat::rgb565, .framebuffer = 0xD0000000UL};
static_assert(ltdc_layer_config_valid(off_the_edge, qvga), "the window leaves the active area");
