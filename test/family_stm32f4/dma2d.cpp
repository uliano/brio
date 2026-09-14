// DMA2D family smoke TU: the Chrom-Art accelerator - the formats, the
// blend model and the refusals, which are pure arithmetic and compile on
// every header of the pack, and the block itself, reached only where the
// part carries one. The accelerator's presence is a WIDER fact than the
// display controller's: two parts of the class have it with no panel
// interface at all.
#include "stm32f4/clock.hpp"
#include "stm32f4/dma2d.hpp"

using namespace brio;

// ---- the reserve's presence facts --------------------------------------------------

static_assert(dma2d_present() == (dma2d_base() != 0u));
static_assert(dma2d_present() == (dma2d_clock_mask() != 0u));
static_assert(dma2d_present() == (dma2d_reset_mask() != 0u));
// Every part with a display has the accelerator; not every part with the
// accelerator has a display.
static_assert(!ltdc_present() || dma2d_present());

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || \
    defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
static_assert(dma2d_present());
static_assert(dma2d_base() == DMA2D_BASE);
static_assert(dma2d_clock_mask() == RCC_AHB1ENR_DMA2DEN);
static_assert(dma2d_reset_mask() == RCC_AHB1RSTR_DMA2DRST);
static_assert(dma2d_irq() == DMA2D_IRQn);
#else
static_assert(!dma2d_present() && dma2d_base() == 0u);
static_assert(dma2d_irq() == NonMaskableInt_IRQn);
#endif

// ---- the eleven input formats and the five output ones -------------------------------

static_assert(dma2d_bits_per_pixel(Dma2dColor::argb8888) == 32);
static_assert(dma2d_bits_per_pixel(Dma2dColor::rgb888) == 24);
static_assert(dma2d_bits_per_pixel(Dma2dColor::rgb565) == 16);
static_assert(dma2d_bits_per_pixel(Dma2dColor::argb1555) == 16);
static_assert(dma2d_bits_per_pixel(Dma2dColor::argb4444) == 16);
static_assert(dma2d_bits_per_pixel(Dma2dColor::l8) == 8);
static_assert(dma2d_bits_per_pixel(Dma2dColor::al44) == 8);
static_assert(dma2d_bits_per_pixel(Dma2dColor::al88) == 16);
static_assert(dma2d_bits_per_pixel(Dma2dColor::l4) == 4);
static_assert(dma2d_bits_per_pixel(Dma2dColor::a8) == 8);
static_assert(dma2d_bits_per_pixel(Dma2dColor::a4) == 4);
static_assert(dma2d_bits_per_pixel(Dma2dOutputColor::argb8888) == 32);
static_assert(dma2d_bits_per_pixel(Dma2dOutputColor::rgb888) == 24);
static_assert(dma2d_bits_per_pixel(Dma2dOutputColor::rgb565) == 16);
static_assert(dma2d_bits_per_pixel(Dma2dOutputColor::argb1555) == 16);
static_assert(dma2d_bits_per_pixel(Dma2dOutputColor::argb4444) == 16);

static_assert(dma2d_format_indexed(Dma2dColor::l8));
static_assert(dma2d_format_indexed(Dma2dColor::l4));
static_assert(dma2d_format_indexed(Dma2dColor::al44));
static_assert(dma2d_format_indexed(Dma2dColor::al88));
static_assert(!dma2d_format_indexed(Dma2dColor::a8));
static_assert(dma2d_format_alpha_only(Dma2dColor::a4));
static_assert(dma2d_format_alpha_only(Dma2dColor::a8));
static_assert(!dma2d_format_alpha_only(Dma2dColor::l8));

// The first eight input codes are the display controller's own eight,
// in the same order (table 53 against 16.7.18), which is why a layer's
// format and a source's format are the same number.
static_assert(static_cast<uint8_t>(Dma2dColor::argb8888) == 0);
static_assert(static_cast<uint8_t>(Dma2dColor::al88) == 7);
static_assert(static_cast<uint8_t>(Dma2dColor::a4) == 10);

// ---- the output colour word (11.5.15) ------------------------------------------------

static_assert(dma2d_output_colour(Dma2dOutputColor::argb8888, 0x11223344UL) == 0x11223344UL);
static_assert(dma2d_output_colour(Dma2dOutputColor::rgb888, 0x11223344UL) == 0x00223344UL);
static_assert(dma2d_output_colour(Dma2dOutputColor::rgb565, 0xFFFF0000UL) == 0xF800u);
static_assert(dma2d_output_colour(Dma2dOutputColor::rgb565, 0xFF00FF00UL) == 0x07E0u);
static_assert(dma2d_output_colour(Dma2dOutputColor::rgb565, 0xFF0000FFUL) == 0x001Fu);
static_assert(dma2d_output_colour(Dma2dOutputColor::argb1555, 0xFFFF0000UL) == 0xFC00u);
static_assert(dma2d_output_colour(Dma2dOutputColor::argb1555, 0x00FF0000UL) == 0x7C00u);
static_assert(dma2d_output_colour(Dma2dOutputColor::argb4444, 0xFFFF0000UL) == 0xFF00u);

// ---- the alpha and the blend (11.3.4, 11.3.11) ---------------------------------------

static_assert(dma2d_source_alpha(Dma2dAlpha::keep, 0x40, 0xFF) == 0x40);
static_assert(dma2d_source_alpha(Dma2dAlpha::replace, 0x40, 0x80) == 0x80);
static_assert(dma2d_source_alpha(Dma2dAlpha::multiply, 0xFF, 0x80) == 0x80);
static_assert(dma2d_source_alpha(Dma2dAlpha::multiply, 0x80, 0x80) == 64);

// An opaque foreground over anything is itself; a transparent one leaves
// the background; alpha out is the union of the two.
static_assert(dma2d_blend_alpha(255, 255) == 255);
static_assert(dma2d_blend_alpha(0, 255) == 255);
static_assert(dma2d_blend_alpha(0, 0) == 0);
static_assert(dma2d_blend_channel(255, 255, 100, 50) == 100);
static_assert(dma2d_blend_channel(0, 255, 100, 50) == 50);
static_assert(dma2d_blend_channel(0, 0, 100, 50) == 0);
// Half over opaque: the divisions rounded to the nearest lower integer.
static_assert(dma2d_blend_channel(128, 255, 200, 40) == 120);

// ---- the refusals --------------------------------------------------------------------

constexpr Dma2dArea small{.pixels = 16, .lines = 8};

static_assert(dma2d_area_valid(small));
static_assert(!dma2d_area_valid(Dma2dArea{.pixels = 0, .lines = 8}));
static_assert(!dma2d_area_valid(Dma2dArea{.pixels = 16, .lines = 0}));
// PL is fourteen bits.
static_assert(dma2d_area_valid(Dma2dArea{.pixels = 0x3FFF, .lines = 1}));
static_assert(!dma2d_area_valid(Dma2dArea{.pixels = 0x4000, .lines = 1}));

static_assert(dma2d_alignment(32) == 4 && dma2d_alignment(16) == 2);
static_assert(dma2d_alignment(24) == 1 && dma2d_alignment(8) == 1 && dma2d_alignment(4) == 1);

constexpr Dma2dSource word_source{.address = 0x2000'0000UL, .format = Dma2dColor::argb8888};
static_assert(dma2d_source_valid(word_source, small));
// 11.5.4: a 32-bit format wants a 32-bit aligned address.
static_assert(!dma2d_source_valid(
    Dma2dSource{.address = 0x2000'0002UL, .format = Dma2dColor::argb8888}, small));
static_assert(!dma2d_source_valid(
    Dma2dSource{.address = 0x2000'0001UL, .format = Dma2dColor::rgb565}, small));
// ... and a 24-bit one takes any byte, because three bytes have no
// wider alignment than one.
static_assert(dma2d_source_valid(
    Dma2dSource{.address = 0x2000'0001UL, .format = Dma2dColor::rgb888}, small));
// 11.3.11: with a 4-bit format both the pixel count and the line offset
// must be even.
static_assert(dma2d_source_valid(Dma2dSource{.address = 0x2000'0000UL,
                                             .line_offset = 4,
                                             .format = Dma2dColor::l4},
                                 small));
static_assert(!dma2d_source_valid(Dma2dSource{.address = 0x2000'0000UL,
                                              .line_offset = 3,
                                              .format = Dma2dColor::l4},
                                  small));
static_assert(!dma2d_source_valid(Dma2dSource{.address = 0x2000'0000UL,
                                              .format = Dma2dColor::a4},
                                  Dma2dArea{.pixels = 15, .lines = 8}));
// The offsets are fourteen bits.
static_assert(!dma2d_source_valid(
    Dma2dSource{.address = 0x2000'0000UL, .line_offset = 0x4000, .format = Dma2dColor::rgb565},
    small));
// CS holds the count minus one in eight bits, so 256 entries is the most
// there is.
static_assert(dma2d_source_valid(Dma2dSource{.address = 0x2000'0000UL,
                                             .format = Dma2dColor::l8,
                                             .clut_address = 0x2000'1000UL,
                                             .clut_entries = 256},
                                 small));
static_assert(!dma2d_source_valid(Dma2dSource{.address = 0x2000'0000UL,
                                              .format = Dma2dColor::l8,
                                              .clut_address = 0x2000'1000UL,
                                              .clut_entries = 257},
                                  small));
static_assert(!dma2d_source_valid(Dma2dSource{.address = 0x2000'0000UL,
                                              .format = Dma2dColor::l8,
                                              .clut_address = 0x2000'1002UL,
                                              .clut_entries = 16},
                                  small));

static_assert(dma2d_output_valid(
    Dma2dOutput{.address = 0x2000'0000UL, .format = Dma2dOutputColor::argb8888}));
static_assert(!dma2d_output_valid(
    Dma2dOutput{.address = 0x2000'0002UL, .format = Dma2dOutputColor::argb8888}));
static_assert(!dma2d_output_valid(Dma2dOutput{
    .address = 0x2000'0000UL, .line_offset = 0x4000, .format = Dma2dOutputColor::rgb565}));

// ---- the block, where it exists ----------------------------------------------------

#if defined(DMA2D_BASE)

void block() {
    (void)&Dma2d::regs();
    Dma2d::init();
    Dma2d::clock(true);
    (void)Dma2d::clock();
    Dma2d::reset();
    Dma2d::enable_interrupt();
    Dma2d::disable_interrupt();
    static_assert(Dma2d::irq_line == DMA2D_IRQn);

    (void)Dma2d::busy();
    (void)Dma2d::wait();
    (void)Dma2d::wait(16);
    Dma2d::abort();
    Dma2d::suspend(true);
    (void)Dma2d::suspended();
    Dma2d::suspend(false);

    Dma2d::interrupt(Dma2dEvent::transfer_complete, true);
    Dma2d::interrupt(Dma2dEvent::transfer_error, true);
    Dma2d::interrupt(Dma2dEvent::watermark, false);
    Dma2d::interrupt(Dma2dEvent::clut_access_error, false);
    Dma2d::interrupt(Dma2dEvent::clut_transfer_complete, false);
    Dma2d::interrupt(Dma2dEvent::configuration_error, true);
    (void)Dma2d::interrupt(Dma2dEvent::transfer_complete);
    (void)Dma2d::flag(Dma2dEvent::configuration_error);
    (void)Dma2d::flags();
    Dma2d::clear(Dma2dEvent::transfer_complete);
    Dma2d::clear(0x3Fu);
    (void)Dma2d::isr();
    Dma2d::watermark(4);
    (void)Dma2d::watermark();
    Dma2d::dead_time(8, true);
    (void)Dma2d::dead_time();
    (void)Dma2d::dead_time_enabled();

    constexpr Dma2dOutput out{
        .address = 0xD000'0000UL, .line_offset = 0, .format = Dma2dOutputColor::rgb565};
    constexpr Dma2dOutput deep{
        .address = 0xD001'0000UL, .line_offset = 0, .format = Dma2dOutputColor::argb8888};
    constexpr Dma2dSource src{.address = 0x2000'0000UL, .format = Dma2dColor::rgb565};
    constexpr Dma2dSource indexed{.address = 0x2000'0000UL,
                                  .format = Dma2dColor::l8,
                                  .clut_address = 0x2000'1000UL,
                                  .clut_entries = 16};

    (void)Dma2d::fill(out, 0xFFFF0000UL, small);
    (void)Dma2d::copy(src, out, small);
    (void)Dma2d::convert(src, deep, small);
    (void)Dma2d::blend(src, src, deep, small);
    (void)Dma2d::start(Dma2dMode::register_to_memory, small);
    (void)Dma2d::load_clut(indexed);
    (void)Dma2d::load_clut(indexed, false);
    (void)Dma2d::clut_busy();
    (void)Dma2d::wait_clut(true, 16);
    Dma2d::clut_entry(0, 0xFF010203UL);
    Dma2d::clut_entry(255, 0, false);
    (void)Dma2d::clut_word(0);
    (void)Dma2d::clut_word(1, false);
}

// The six events sit at the same bit in ISR and IFCR, and eight higher
// in CR's enables.
static_assert(dma2d_event_mask(Dma2dEvent::transfer_error) == DMA2D_ISR_TEIF);
static_assert(dma2d_event_mask(Dma2dEvent::transfer_error) == DMA2D_IFCR_CTEIF);
static_assert(dma2d_event_mask(Dma2dEvent::transfer_complete) == DMA2D_ISR_TCIF);
static_assert(dma2d_event_mask(Dma2dEvent::transfer_complete) == DMA2D_IFCR_CTCIF);
static_assert(dma2d_event_mask(Dma2dEvent::watermark) == DMA2D_ISR_TWIF);
static_assert(dma2d_event_mask(Dma2dEvent::watermark) == DMA2D_IFCR_CTWIF);
static_assert(dma2d_event_mask(Dma2dEvent::clut_access_error) == DMA2D_ISR_CAEIF);
static_assert(dma2d_event_mask(Dma2dEvent::clut_access_error) == DMA2D_IFCR_CAECIF);
static_assert(dma2d_event_mask(Dma2dEvent::clut_transfer_complete) == DMA2D_ISR_CTCIF);
static_assert(dma2d_event_mask(Dma2dEvent::clut_transfer_complete) == DMA2D_IFCR_CCTCIF);
static_assert(dma2d_event_mask(Dma2dEvent::configuration_error) == DMA2D_ISR_CEIF);
static_assert(dma2d_event_mask(Dma2dEvent::configuration_error) == DMA2D_IFCR_CCEIF);
static_assert((dma2d_event_mask(Dma2dEvent::transfer_error) << DMA2D_CR_TEIE_Pos) == DMA2D_CR_TEIE);
static_assert((dma2d_event_mask(Dma2dEvent::configuration_error) << DMA2D_CR_TEIE_Pos) ==
              DMA2D_CR_CEIE);

// The four modes are the register's own codes (11.5.1).
static_assert(static_cast<uint8_t>(Dma2dMode::memory_to_memory) == 0);
static_assert(static_cast<uint8_t>(Dma2dMode::memory_to_memory_pfc) == 1);
static_assert(static_cast<uint8_t>(Dma2dMode::memory_to_memory_blend) == 2);
static_assert(static_cast<uint8_t>(Dma2dMode::register_to_memory) == 3);

#endif   // DMA2D_BASE
