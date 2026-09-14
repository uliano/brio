// LTDC family smoke TU: the LCD-TFT display controller - the pixel
// packers and the timing arithmetic, which are pure numbers and compile
// on every header of the pack, and the block, its two layers and the
// pixel clock's PLL, which are reached only where the part has a
// display interface.
#include <type_traits>

#include "stm32f4/clock.hpp"
#include "stm32f4/ltdc.hpp"

using namespace brio;

// ---- the reserve's presence facts --------------------------------------------------

static_assert(ltdc_present() == (ltdc_base() != 0u));
static_assert(ltdc_present() == (ltdc_clock_mask() != 0u));
static_assert(ltdc_present() == (ltdc_reset_mask() != 0u));
static_assert(ltdc_layers() == (ltdc_present() ? 2u : 0u));
static_assert(ltdc_layer_base(0) == 0u && ltdc_layer_base(3) == 0u);
static_assert(ltdc_present() == (ltdc_layer_base(1) != 0u));
static_assert(ltdc_present() == (ltdc_layer_base(2) != 0u));
// The pixel clock's PLL is a wider fact than the controller: the class
// that carries a display declares PLLSAI's R output on every part of
// it, display or no display, and the part that has this PLL for its
// audio interface alone declares neither R nor the LCD divider.
static_assert(!ltdc_present() || pllsai_present());
static_assert(!ltdc_present() || pllsai_has_r());
static_assert(!ltdc_present() || pllsai_has_lcd_divider());
static_assert(pllsai_has_r() == pllsai_has_lcd_divider());
static_assert(!pllsai_has_r() || pllsai_present());

#if defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
// The parts with a display interface.
static_assert(ltdc_present());
static_assert(ltdc_base() == LTDC_BASE);
static_assert(ltdc_layer_base(1) == LTDC_Layer1_BASE);
static_assert(ltdc_layer_base(2) == LTDC_Layer2_BASE);
// 16.7.14: the two layer blocks are 0x80 apart.
static_assert(ltdc_layer_base(2) - ltdc_layer_base(1) == 0x80u);
static_assert(ltdc_clock_mask() == RCC_APB2ENR_LTDCEN);
static_assert(ltdc_reset_mask() == RCC_APB2RSTR_LTDCRST);
static_assert(ltdc_irq() == LTDC_IRQn && ltdc_error_irq() == LTDC_ER_IRQn);
static_assert(ltdc_irq() != ltdc_error_irq());
#elif defined(STM32F427xx) || defined(STM32F437xx)
// The same class WITHOUT the display: no controller, and the second
// PLL's R output declared all the same.
static_assert(!ltdc_present() && ltdc_base() == 0u);
static_assert(pllsai_present() && pllsai_has_r());
#elif defined(STM32F446xx)
// The second PLL for the audio interface alone.
static_assert(!ltdc_present());
static_assert(pllsai_present() && !pllsai_has_r() && !pllsai_has_lcd_divider());
#else
// No display, no second PLL.
static_assert(!ltdc_present() && !pllsai_present());
static_assert(!pllsai_has_r() && !pllsai_has_lcd_divider());
static_assert(ltdc_irq() == NonMaskableInt_IRQn);
#endif

// ---- the pixel packers, everywhere -------------------------------------------------

static_assert(argb8888(0x12, 0x34, 0x56, 0x78) == 0x12345678UL);
static_assert(rgb888(0x34, 0x56, 0x78) == 0x00345678UL);
// The top bits of each channel, the rest dropped: 0xFF -> all ones.
static_assert(rgb565(0xFF, 0xFF, 0xFF) == 0xFFFFu);
static_assert(rgb565(0, 0, 0) == 0x0000u);
static_assert(rgb565(0xFF, 0, 0) == 0xF800u);
static_assert(rgb565(0, 0xFF, 0) == 0x07E0u);
static_assert(rgb565(0, 0, 0xFF) == 0x001Fu);
static_assert(argb1555(true, 0xFF, 0, 0) == 0xFC00u);
static_assert(argb1555(false, 0, 0, 0xFF) == 0x001Fu);
static_assert(argb4444(0xFF, 0xFF, 0, 0) == 0xFF00u);
static_assert(argb4444(0, 0, 0xFF, 0xFF) == 0x00FFu);

// ---- the formats -------------------------------------------------------------------

static_assert(ltdc_bytes_per_pixel(LtdcPixelFormat::argb8888) == 4);
static_assert(ltdc_bytes_per_pixel(LtdcPixelFormat::rgb888) == 3);
static_assert(ltdc_bytes_per_pixel(LtdcPixelFormat::rgb565) == 2);
static_assert(ltdc_bytes_per_pixel(LtdcPixelFormat::argb1555) == 2);
static_assert(ltdc_bytes_per_pixel(LtdcPixelFormat::argb4444) == 2);
static_assert(ltdc_bytes_per_pixel(LtdcPixelFormat::l8) == 1);
static_assert(ltdc_bytes_per_pixel(LtdcPixelFormat::al44) == 1);
static_assert(ltdc_bytes_per_pixel(LtdcPixelFormat::al88) == 2);
static_assert(ltdc_format_indexed(LtdcPixelFormat::l8));
static_assert(ltdc_format_indexed(LtdcPixelFormat::al44));
static_assert(ltdc_format_indexed(LtdcPixelFormat::al88));
static_assert(!ltdc_format_indexed(LtdcPixelFormat::rgb565));
// 16.4.2: an AL44 table's sixteen entries sit at the replicated
// addresses, so entry 1 is 0x11 and entry 15 is 0xFF.
static_assert(ltdc_al44_clut_address(0) == 0x00u);
static_assert(ltdc_al44_clut_address(1) == 0x11u);
static_assert(ltdc_al44_clut_address(15) == 0xFFu);

// ---- the timing arithmetic ---------------------------------------------------------

// The panel a board of this family carries: 240x320, the timings ST's
// own board support states for it.
constexpr LtdcTiming qvga{.hsync = 10,
                          .hbp = 20,
                          .width = 240,
                          .hfp = 10,
                          .vsync = 2,
                          .vbp = 2,
                          .height = 320,
                          .vfp = 4};

static_assert(ltdc_timing_valid(qvga));
static_assert(ltdc_total_width(qvga) == 280);
static_assert(ltdc_total_height(qvga) == 328);
static_assert(ltdc_frame_pixels(qvga) == 91'840UL);
// 6 MHz over 91840 pixels: 65.331 Hz, in millihertz.
static_assert(ltdc_frame_rate_mhz(6'000'000u, qvga) == 65'331u);
// One RGB565 layer of it: 240 x 320 x 2 bytes 65.331 times a second.
static_assert(ltdc_fetch_bytes_per_second(6'000'000u, qvga, LtdcPixelFormat::rgb565) ==
              10'034'841UL);
static_assert(ltdc_fetch_bytes_per_second(6'000'000u, qvga, LtdcPixelFormat::argb8888) ==
              20'069'683UL);

// The example of 16.4.1, whose four register words the chapter states.
constexpr LtdcTiming vga{.hsync = 8, .hbp = 7, .width = 640, .hfp = 5,
                         .vsync = 4, .vbp = 2, .height = 480, .vfp = 1};
static_assert(ltdc_timing_valid(vga));
static_assert(ltdc_total_width(vga) == 660 && ltdc_total_height(vga) == 487);

// Every field is one less than a count, so no count may be zero; the
// active area stops at 1024x768.
static_assert(!ltdc_timing_valid(LtdcTiming{}));
static_assert(!ltdc_timing_valid(LtdcTiming{.hsync = 0, .hbp = 1, .width = 240, .hfp = 1,
                                            .vsync = 1, .vbp = 1, .height = 320, .vfp = 1}));
static_assert(!ltdc_timing_valid(LtdcTiming{.hsync = 1, .hbp = 1, .width = 1025, .hfp = 1,
                                            .vsync = 1, .vbp = 1, .height = 320, .vfp = 1}));
static_assert(!ltdc_timing_valid(LtdcTiming{.hsync = 1, .hbp = 1, .width = 240, .hfp = 1,
                                            .vsync = 1, .vbp = 1, .height = 769, .vfp = 1}));

static_assert(ltdc_window_valid(LtdcWindow{0, 0, 240, 320}, qvga));
static_assert(ltdc_window_valid(LtdcWindow{100, 100, 140, 220}, qvga));
static_assert(!ltdc_window_valid(LtdcWindow{1, 0, 240, 320}, qvga));
static_assert(!ltdc_window_valid(LtdcWindow{0, 0, 0, 320}, qvga));

// 16.7.23: the line length register holds the bytes PLUS THREE, and the
// chapter's own two examples say so.
static_assert(ltdc_line_length(256, LtdcPixelFormat::rgb565) == 0x203u);
static_assert(ltdc_line_length(320, LtdcPixelFormat::rgb888) == 0x3C3u);

constexpr LtdcLayerConfig full{.window = {0, 0, 240, 320},
                               .format = LtdcPixelFormat::rgb565,
                               .framebuffer = 0xD000'0000UL};
static_assert(ltdc_layer_config_valid(full, qvga));
// A CLUT asked for on a format that has no index.
static_assert(!ltdc_layer_config_valid(
    LtdcLayerConfig{.window = {0, 0, 240, 320},
                    .format = LtdcPixelFormat::rgb565,
                    .framebuffer = 0xD000'0000UL,
                    .clut = true},
    qvga));
// A pitch narrower than the line it is the pitch of.
static_assert(!ltdc_layer_config_valid(
    LtdcLayerConfig{.window = {0, 0, 240, 320},
                    .format = LtdcPixelFormat::rgb565,
                    .framebuffer = 0xD000'0000UL,
                    .pitch = 100},
    qvga));

// ---- the blending model (16.7.21) --------------------------------------------------

// The chapter's own worked example: constant alpha 240, layer 128,
// background 48, the result 123.
static_assert(ltdc_blend_channel(LtdcBlend1::constant_alpha,
                                 LtdcBlend2::one_minus_constant_alpha, 240, 255, 128, 48) == 123);
// A fully opaque layer over anything is itself.
static_assert(ltdc_blend_channel(LtdcBlend1::constant_alpha,
                                 LtdcBlend2::one_minus_constant_alpha, 255, 255, 200, 40) == 200);
// A fully transparent one leaves what is under it.
static_assert(ltdc_blend_channel(LtdcBlend1::constant_alpha,
                                 LtdcBlend2::one_minus_constant_alpha, 0, 255, 200, 40) == 40);
// The pixel alpha multiplies the constant one.
static_assert(ltdc_blend_channel(LtdcBlend1::pixel_alpha_x_constant,
                                 LtdcBlend2::one_minus_pixel_alpha_x_constant, 255, 128, 200,
                                 40) == 120);

// The register codes are the chapter's, which is what makes the reset
// value 0x0607 mean "pixel alpha times constant, and one minus it".
static_assert(static_cast<uint8_t>(LtdcBlend1::constant_alpha) == 4);
static_assert(static_cast<uint8_t>(LtdcBlend1::pixel_alpha_x_constant) == 6);
static_assert(static_cast<uint8_t>(LtdcBlend2::one_minus_constant_alpha) == 5);
static_assert(static_cast<uint8_t>(LtdcBlend2::one_minus_pixel_alpha_x_constant) == 7);

// ---- the pixel clock's arithmetic --------------------------------------------------

// 8 MHz on HSE with the main PLL's M of 4 - the VCO input the system
// clock already fixed - makes 6 MHz exactly: N 60, R 5, the LCD divider
// 4.
constexpr LcdClockConfig six_mhz = lcd_clock_config_for(8'000'000u, 6'000'000u, 4);
static_assert(six_mhz.pll.n == 60 && six_mhz.pll.r == 5);
static_assert(six_mhz.divider == LcdClockDivider::div4);
static_assert(pllsai_r_hz(8'000'000u, six_mhz.pll, 4) == 24'000'000u);
static_assert(lcd_clock_hz(pllsai_r_hz(8'000'000u, six_mhz.pll, 4), six_mhz.divider) ==
              6'000'000u);
// A root the main PLL's divider leaves outside the VCO's input window
// has no triple at all: this PLL has no divider of its own.
static_assert(lcd_clock_config_for(25'000'000u, 6'000'000u, 4).pll.n == 0);
static_assert(lcd_clock_config_for(8'000'000u, 6'000'000u, 0).pll.n == 0);
static_assert(lcd_clock_config_for(8'000'000u, 0u, 4).pll.n == 0);
// The four divider codes of 6.3.25.
static_assert(lcd_divider_code(LcdClockDivider::div2) == 0);
static_assert(lcd_divider_code(LcdClockDivider::div4) == 1);
static_assert(lcd_divider_code(LcdClockDivider::div8) == 2);
static_assert(lcd_divider_code(LcdClockDivider::div16) == 3);
static_assert(pllsai_r_hz(8'000'000u, PllSaiConfig{}, 4) == 0u);

// ---- a framebuffer as a type -------------------------------------------------------

void surface() {
    static uint16_t pixels[16 * 8];
    const LtdcFramebuffer<uint16_t> fb{pixels, 16, 8, 0};
    fb.fill(rgb565(0, 0, 0));
    fb.fill_rect(2, 2, 4, 4, rgb565(255, 255, 255));
    fb.write(0, 0, rgb565(255, 0, 0));
    fb.write(100, 100, 0);   // clipped, not written
    (void)fb.line(3);
    (void)fb.bytes();
    (void)fb.line_bytes();
    (void)fb.pitch_bytes();
    (void)fb.stride();
    (void)fb.address();

    static uint32_t deep[8 * 4];
    const LtdcFramebuffer<uint32_t> wide{deep, 4, 4, 8};
    wide.fill(argb8888(255, 0, 0, 255));
    static_assert(std::is_same_v<decltype(wide.line(0)), volatile uint32_t*>);
}

// ---- the block and its layers, where they exist -------------------------------------

#if defined(LTDC_BASE)

void block() {
    (void)&Ltdc::regs();
    Ltdc::clock(true);
    (void)Ltdc::clock();
    Ltdc::reset();
    Ltdc::enable_interrupt();
    Ltdc::disable_interrupt();
    Ltdc::enable_error_interrupt();
    Ltdc::disable_error_interrupt();
    static_assert(Ltdc::layers == 2);
    static_assert(Ltdc::irq_line == LTDC_IRQn);
    static_assert(Ltdc::error_irq_line == LTDC_ER_IRQn);

    (void)Ltdc::pixel_clock(8'000'000u, 6'000'000u);
    (void)Ltdc::pixel_clock(six_mhz);
    (void)Ltdc::pixel_clock(8'000'000u);

    (void)Ltdc::timing(qvga);
    (void)Ltdc::timing(vga);
    (void)Ltdc::timing();
    (void)Ltdc::horizontal_back_porch();
    (void)Ltdc::vertical_back_porch();

    Ltdc::enable();
    Ltdc::disable();
    (void)Ltdc::enabled();
    Ltdc::background(1, 2, 3);
    (void)Ltdc::background();
    Ltdc::dither(true);
    (void)Ltdc::dither();
    (void)Ltdc::dither_red_width();
    (void)Ltdc::dither_green_width();
    (void)Ltdc::dither_blue_width();

    Ltdc::reload(LtdcReload::immediate);
    Ltdc::reload(LtdcReload::vertical_blanking);
    (void)Ltdc::reload_pending();
    (void)Ltdc::wait_reload();
    (void)Ltdc::wait_reload(16);

    (void)Ltdc::line_interrupt(320);
    (void)Ltdc::line_interrupt();
    Ltdc::interrupt(LtdcEvent::line, true);
    Ltdc::interrupt(LtdcEvent::fifo_underrun, true);
    Ltdc::interrupt(LtdcEvent::transfer_error, false);
    Ltdc::interrupt(LtdcEvent::register_reload, false);
    (void)Ltdc::interrupt(LtdcEvent::line);
    (void)Ltdc::flag(LtdcEvent::fifo_underrun);
    (void)Ltdc::flags();
    Ltdc::clear(LtdcEvent::line);
    Ltdc::clear(ltdc_event_mask(LtdcEvent::line) | ltdc_event_mask(LtdcEvent::register_reload));
    (void)Ltdc::isr();

    (void)Ltdc::position();
    (void)Ltdc::display_status();
}

// The four events sit at the same bit in all three of IER, ISR and ICR,
// which is what lets one enumeration serve the three registers.
static_assert(ltdc_event_mask(LtdcEvent::line) == LTDC_IER_LIE);
static_assert(ltdc_event_mask(LtdcEvent::line) == LTDC_ISR_LIF);
static_assert(ltdc_event_mask(LtdcEvent::line) == LTDC_ICR_CLIF);
static_assert(ltdc_event_mask(LtdcEvent::fifo_underrun) == LTDC_IER_FUIE);
static_assert(ltdc_event_mask(LtdcEvent::fifo_underrun) == LTDC_ISR_FUIF);
static_assert(ltdc_event_mask(LtdcEvent::fifo_underrun) == LTDC_ICR_CFUIF);
static_assert(ltdc_event_mask(LtdcEvent::transfer_error) == LTDC_IER_TERRIE);
static_assert(ltdc_event_mask(LtdcEvent::transfer_error) == LTDC_ISR_TERRIF);
static_assert(ltdc_event_mask(LtdcEvent::transfer_error) == LTDC_ICR_CTERRIF);
static_assert(ltdc_event_mask(LtdcEvent::register_reload) == LTDC_IER_RRIE);
static_assert(ltdc_event_mask(LtdcEvent::register_reload) == LTDC_ISR_RRIF);
static_assert(ltdc_event_mask(LtdcEvent::register_reload) == LTDC_ICR_CRRIF);

template <uint8_t n>
void one_layer() {
    using L = LtdcLayer<n>;
    (void)&L::regs();
    L::window(LtdcWindow{0, 0, 240, 320});
    (void)L::window();
    L::pixel_format(LtdcPixelFormat::rgb565);
    (void)L::pixel_format();
    L::framebuffer(0xD000'0000UL);
    (void)L::framebuffer();
    (void)L::buffer_line(480, 480);
    (void)L::buffer_lines(320);
    (void)L::buffer_lines();
    L::constant_alpha(255);
    (void)L::constant_alpha();
    L::blending(LtdcBlend1::constant_alpha, LtdcBlend2::one_minus_constant_alpha);
    L::blending(LtdcBlend1::pixel_alpha_x_constant,
                LtdcBlend2::one_minus_pixel_alpha_x_constant);
    L::default_colour(argb8888(0, 0, 0, 0));
    (void)L::default_colour();
    L::colour_key(rgb888(255, 0, 255));
    (void)L::colour_key();
    L::colour_keying(true);
    L::clut_entry(0, 1, 2, 3);
    L::clut_entry(ltdc_al44_clut_address(1), 4, 5, 6);
    L::clut(true);
    (void)L::clut();
    L::enable();
    L::disable();
    (void)L::enabled();
    (void)L::configure(full, qvga);
}

void layers() {
    one_layer<1>();
    one_layer<2>();
}

#endif   // LTDC_BASE
