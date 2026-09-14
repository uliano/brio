// test_stm32f4_ltdc - the reference bench suite for the STM32F4's
// display tier: the LCD-TFT controller reading a frame buffer out of the
// external memory and driving the panel this board carries, and the
// Chrom-Art accelerator filling, converting and blending over the same
// memory - stm32f4/ltdc.hpp and stm32f4/dma2d.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENT IS A PANEL AND EIGHT MEGABYTES OF DRAM, which is why
// this suite builds for the STM32F429I-DISC1 alone: it is the one board
// of this desk with a display interface, a display on it and a memory
// wide enough to hold a frame of it.
//
//   the panel    a 240x320 TFT with an ILI9341 controller, driven in its
//                RGB (DPI) mode over eighteen data lines plus HSYNC,
//                VSYNC, DE and the pixel clock; its own registers are
//                reached over SPI5, whose MOSI pad is its only data line
//   the memory   the 64-Mbit SDRAM on FMC bank 2 at 0xD0000000, brought
//                up by stm32f4/fmc.hpp before a pixel is fetched
//   the clocks   the core at 180 MHz, the memory at HCLK/2 = 90 MHz, and
//                LCD_CLK at 6 MHz off PLLSAI (N 60, R 5, the LCD divider
//                4, from the 8 MHz crystal over the main PLL's M of 4)
//   the pads     AF14 throughout except four that carry their LTDC
//                signal on AF9, from the board's schematic:
//                  R2 PC10  R3 PB0*  R4 PA11  R5 PA12  R6 PB1*  R7 PG6
//                  G2 PA6   G3 PG10* G4 PB10  G5 PB11  G6 PC7   G7 PD3
//                  B2 PD6   B3 PG11  B4 PG12* B5 PA3   B6 PB8   B7 PB9
//                  HSYNC PC6  VSYNC PA4  DE PF10  CLK PG7
//                (* = AF9), with the panel's own select on PC2, its
//                command/data on PD13 and its tearing-effect output on
//                PD11
//
// WHAT A PROGRAM CAN SEE AND WHAT NEEDS EYES. The controller has no
// read path into the panel and this board does not wire the panel's
// own: everything below is judged on what the SILICON reports - the
// position counter walking, the line event landing where it was asked
// for, the FIFO underrun staying clear, the accelerator's output read
// back byte for byte - and the one thing that says the PIXELS are
// right is a human looking at the panel. Letter n is for that look: it
// leaves colour bars and a moving bar on the screen.
//
// THE FRAME BUFFERS LIVE IN THE EXTERNAL MEMORY and the CPU only ever
// WRITES them: ES0206 2.3.5 says a CPU read of FMC-held data with
// interrupts live may be corrupted, while reads by another master are
// not affected - and the readers here are the display controller and
// the accelerator. The one letter that reads a result back byte for
// byte (k) works in internal SRAM for exactly that reason.
//
// What is exercised, letter by letter:
//   a  the two blocks: what this part has, the gates closed at reset,
//      and the register values behind them
//   b  the pixel clock: the PLL's triple, and the frame rate measured
//      against SysTick
//   c  what the drivers refuse, and that a refusal writes nothing
//   d  the panel taken into its RGB mode over SPI5, and its
//      tearing-effect output as the one answer it can give
//   e  the timing registers: the panel's numbers in, the same numbers
//      back, and the chapter's own example decoded
//   f  the position counter walking a whole frame, and what a read of
//      it costs
//   g  the line event at the programmed line, once a frame
//   h  the shadow registers: the two reloads and what a read returns
//      before one
//   i  one layer of 16-bit pixels out of the external memory: the
//      underrun flag over a hundred frames
//   j  THE BANDWIDTH: one and two layers, 16 and 32 bits a pixel, with
//      and without the accelerator filling the same memory
//   k  the accelerator's four modes, byte-exact against a model
//   l  the colour tables: the accelerator's, loaded both ways
//   m  the layer geometry and the blending registers
//   n  colour bars and a moving bar, for a human to look at
//
// build: boards = f429zi
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma2d.hpp"
#include "stm32f4/fmc.hpp"
#include "stm32f4/ltdc.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the console ---------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7},
                                .rx = {'A', 10, PinFunction::af7}};
using Serial = Uart<1, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- the external memory ----------------------------------------------------------

using Sdram = FmcSdram<2>;

/// The board's device, in the words its own suite states it in: 4096
/// rows x 256 columns x 4 internal banks x 16 bits.
constexpr SdramConfig sdram_geometry{.columns = SdramColumns::eight,
                                     .rows = SdramRows::twelve,
                                     .width = SdramWidth::bits16,
                                     .internal_banks = SdramInternalBanks::four,
                                     .cas = SdramCas::three,
                                     .write_protect = false,
                                     .clock = SdramClock::hclk_div2,
                                     .read_burst = true,
                                     .read_pipe = SdramPipe::none};

constexpr SdramTiming sdram_timing{.load_mode_to_active = 2,
                                   .exit_self_refresh = 7,
                                   .self_refresh = 4,
                                   .row_cycle = 6,
                                   .write_recovery = 2,
                                   .row_precharge = 2,
                                   .row_to_column = 2};

constexpr uint32_t sdram_hz = sdram_clock_hz(SysClock::hz, sdram_geometry.clock);
constexpr uint32_t sdram_refresh =
    sdram_refresh_count(sdram_hz, 64'000u, 4096u);

constexpr SdramInit sdram_boot{.power_up_us = 100,
                               .refresh_cycles = 8,
                               .mode_register = sdram_mode_register(SdramCas::three),
                               .refresh_count = static_cast<uint16_t>(sdram_refresh),
                               .both_banks = false};

/// The memory's pads, per port, exactly as the schematic wires them.
void claim_sdram_pads() {
    constexpr PinConfig cfg{
        .pull = PinPull::up, .open_drain = false, .speed = PinSpeed::very_high};
    Port<'B'>::configure_mask((1u << 5) | (1u << 6), PinMode::alternate, cfg, PinFunction::af12);
    Port<'C'>::configure_mask(1u << 0, PinMode::alternate, cfg, PinFunction::af12);
    Port<'D'>::configure_mask((1u << 0) | (1u << 1) | (1u << 8) | (1u << 9) | (1u << 10) |
                                  (1u << 14) | (1u << 15),
                              PinMode::alternate, cfg, PinFunction::af12);
    Port<'E'>::configure_mask((1u << 0) | (1u << 1) | 0xFF80u, PinMode::alternate, cfg,
                              PinFunction::af12);
    Port<'F'>::configure_mask(0x003Fu | (1u << 11) | 0xF000u, PinMode::alternate, cfg,
                              PinFunction::af12);
    Port<'G'>::configure_mask((1u << 0) | (1u << 1) | (1u << 4) | (1u << 5) | (1u << 8) |
                                  (1u << 15),
                              PinMode::alternate, cfg, PinFunction::af12);
}

// ---- the panel ------------------------------------------------------------------------

constexpr uint16_t panel_width = 240;
constexpr uint16_t panel_height = 320;

/// The timings ST's own board support states for this panel, in the
/// panel's words rather than accumulated.
constexpr LtdcTiming panel_timing{.hsync = 10,
                                  .hbp = 20,
                                  .width = panel_width,
                                  .hfp = 10,
                                  .vsync = 2,
                                  .vbp = 2,
                                  .height = panel_height,
                                  .vfp = 4,
                                  .hsync_polarity = LtdcPolarity::active_low,
                                  .vsync_polarity = LtdcPolarity::active_low,
                                  .de_polarity = LtdcPolarity::active_low,
                                  .clock_edge = LtdcClockEdge::direct};

constexpr uint32_t pixel_hz = 6'000'000u;
constexpr LcdClockConfig pixel_pll = lcd_clock_config_for(8'000'000u, pixel_hz, 4);
static_assert(pixel_pll.pll.n == 60 && pixel_pll.pll.r == 5);
static_assert(pixel_pll.divider == LcdClockDivider::div4);

constexpr uint32_t frame_pixels = ltdc_frame_pixels(panel_timing);
constexpr uint32_t frame_rate_mhz = ltdc_frame_rate_mhz(pixel_hz, panel_timing);
/// The frame period in microseconds, from the arithmetic alone.
constexpr uint32_t frame_period_us =
    static_cast<uint32_t>((static_cast<uint64_t>(frame_pixels) * 1'000'000u) / pixel_hz);

/// The panel's own controller, on SPI5's single data line.
constexpr SpiPins spi5_pins{.sck = {'F', 7, PinFunction::af5},
                            .miso = {'F', 8, PinFunction::af5},
                            .mosi = {'F', 9, PinFunction::af5}};
using S = Spi<5>;
using Host = SpiHost<5, spi5_pins>;
using LcdCs = Pin<'C', 2>;
using LcdDcx = Pin<'D', 13>;
using LcdTe = Pin<'D', 11>;
using GyroCs = Pin<'C', 1>;

/// The display's pads, split by alternate function: four of the
/// twenty-two carry their LTDC signal on AF9 and the rest on AF14
/// (DS10693 table 12).
void claim_panel_pads() {
    constexpr PinConfig cfg{.pull = PinPull::none, .open_drain = false, .speed = PinSpeed::high};
    Port<'A'>::configure_mask((1u << 3) | (1u << 4) | (1u << 6) | (1u << 11) | (1u << 12),
                              PinMode::alternate, cfg, PinFunction::af14);
    Port<'B'>::configure_mask((1u << 8) | (1u << 9) | (1u << 10) | (1u << 11), PinMode::alternate,
                              cfg, PinFunction::af14);
    Port<'C'>::configure_mask((1u << 6) | (1u << 7) | (1u << 10), PinMode::alternate, cfg,
                              PinFunction::af14);
    Port<'D'>::configure_mask((1u << 3) | (1u << 6), PinMode::alternate, cfg, PinFunction::af14);
    Port<'F'>::configure_mask(1u << 10, PinMode::alternate, cfg, PinFunction::af14);
    Port<'G'>::configure_mask((1u << 6) | (1u << 7) | (1u << 11), PinMode::alternate, cfg,
                              PinFunction::af14);
    Port<'B'>::configure_mask((1u << 0) | (1u << 1), PinMode::alternate, cfg, PinFunction::af9);
    Port<'G'>::configure_mask((1u << 10) | (1u << 12), PinMode::alternate, cfg, PinFunction::af9);
}

// ---- the frame buffers ------------------------------------------------------------

/// Three surfaces in the external memory, a megabyte apart so that each
/// starts on a row of its own and no two share one: a 16-bit one for the
/// picture and two 32-bit ones for the two-layer measurement. The fourth
/// megabyte is the accelerator's scratch, which nothing displays.
constexpr uint32_t sdram_base = Sdram::base;
constexpr uint32_t fb16_offset = 0x0000'0000u;
constexpr uint32_t fb32a_offset = 0x0010'0000u;
constexpr uint32_t fb32b_offset = 0x0020'0000u;
constexpr uint32_t fb8_offset = 0x0030'0000u;
constexpr uint32_t scratch_offset = 0x0040'0000u;

const LtdcFramebuffer<uint16_t> fb16{Sdram::at<uint16_t>(fb16_offset), panel_width, panel_height,
                                     0};
const LtdcFramebuffer<uint32_t> fb32a{Sdram::at<uint32_t>(fb32a_offset), panel_width,
                                      panel_height, 0};
const LtdcFramebuffer<uint32_t> fb32b{Sdram::at<uint32_t>(fb32b_offset), panel_width,
                                      panel_height, 0};

// ---- the rulers -----------------------------------------------------------------------

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

uint32_t systick_period() { return SysTick->LOAD + 1u; }
uint32_t cycle_stamp() { return SysTick->VAL; }

/// SysTick counts DOWN, so a later sample is a smaller number - unless
/// the period wrapped, which at 1000 Hz means the span was longer than a
/// millisecond and is not to be trusted.
uint32_t val_delta(uint32_t first, uint32_t second) {
    return first >= second ? first - second : first + systick_period() - second;
}

uint32_t millis() { return P::now(); }

void wait_ms(uint32_t ms) {
    const uint32_t start = millis();
    while (millis() - start < ms) {
    }
}

// ---- what the registers held before this program touched them --------------------------

/// The controller's registers, as one capture. The suite takes TWO of
/// them - one with the block clocked by the APB alone and one with the
/// pixel clock running - because 16.3.2's three clock domains turn out
/// to be visible from the bus.
struct Registers {
    uint32_t gcr = 0, sscr = 0, bpcr = 0, awcr = 0, twcr = 0;
    uint32_t srcr = 0, bccr = 0, ier = 0, isr = 0, lipcr = 0, cdsr = 0;
    uint32_t l1cr = 0, l1cacr = 0, l1bfcr = 0, l1pfcr = 0, l1dccr = 0;
    uint32_t l1cfbar = 0, l1cfblr = 0, l1cfblnr = 0;
    uint32_t l2cr = 0, l2cacr = 0, l2bfcr = 0;
};

Registers read_registers() {
    Registers r{};
    const LTDC_TypeDef& b = Ltdc::regs();
    r.gcr = b.GCR;
    r.sscr = b.SSCR;
    r.bpcr = b.BPCR;
    r.awcr = b.AWCR;
    r.twcr = b.TWCR;
    r.srcr = b.SRCR;
    r.bccr = b.BCCR;
    r.ier = b.IER;
    r.isr = b.ISR;
    r.lipcr = b.LIPCR;
    r.cdsr = b.CDSR;
    r.l1cr = LtdcLayer<1>::regs().CR;
    r.l1cacr = LtdcLayer<1>::regs().CACR;
    r.l1bfcr = LtdcLayer<1>::regs().BFCR;
    r.l1pfcr = LtdcLayer<1>::regs().PFCR;
    r.l1dccr = LtdcLayer<1>::regs().DCCR;
    r.l1cfbar = LtdcLayer<1>::regs().CFBAR;
    r.l1cfblr = LtdcLayer<1>::regs().CFBLR;
    r.l1cfblnr = LtdcLayer<1>::regs().CFBLNR;
    r.l2cr = LtdcLayer<2>::regs().CR;
    r.l2cacr = LtdcLayer<2>::regs().CACR;
    r.l2bfcr = LtdcLayer<2>::regs().BFCR;
    return r;
}

struct BootState {
    bool ltdc_gate = false;
    bool dma2d_gate = false;
    bool pllsai_on = false;
    /// With the APB2 gate open and no pixel clock at all.
    Registers dark{};
    /// With the pixel clock locked, and still nothing written.
    Registers clocked{};
    uint32_t d2cr = 0, d2isr = 0, d2amtcr = 0, d2opfccr = 0;
    bool sdram_up = false;
    bool clock_up = false;
    bool panel_up = false;
    uint16_t panel_bytes = 0;
};
BootState boot;

// ---- the line event ---------------------------------------------------------------------

volatile uint32_t line_events = 0;
volatile uint32_t reload_events = 0;
volatile uint32_t error_events = 0;
volatile uint16_t line_event_y = 0;

// =============================================================================
// The panel's own controller, over SPI5
// =============================================================================

/// SPI5 as the panel wants it: one data line out, mode 0, and a rate the
/// controller's serial interface takes without a wire on the bench.
void panel_bus() {
    Nvic::disable(S::irq);
    (void)S::configure(SpiConfig{.role = SpiRole::host,
                                 .mode = SpiMode::mode0,
                                 .clock = SpiClock::div32,
                                 .nss = SpiNss::software,
                                 .direction = SpiDirection::half_duplex_out});
    Pin<'F', 9>::function(PinFunction::af5, {.speed = PinSpeed::very_high});
    S::enable();
}

/// One frame out on the single line, waited to the last edge.
void panel_frame(uint8_t v) {
    S::data(v);
    uint32_t spins = 400'000u;
    while (!S::tx_empty() && spins-- != 0u) {
    }
    spins = 400'000u;
    while (S::busy() && spins-- != 0u) {
    }
}

/// One command and its parameters: the command byte with the
/// command/data pad LOW, the parameters with it high, both inside one
/// selection.
void panel_write(uint8_t command, const uint8_t* params, uint8_t count) {
    LcdDcx::clear();
    LcdCs::clear();
    panel_frame(command);
    LcdDcx::set();
    for (uint8_t i = 0; i < count; ++i) {
        panel_frame(params[i]);
    }
    LcdCs::set();
}

/// ST's own initialization sequence for this panel, as its board
/// support package writes it, encoded as {command, count, parameters}.
/// The two that matter for this chapter are 0xB0 - the RGB interface
/// signal control, which is what makes the controller take its pixels
/// from the parallel bus instead of from its own memory - and 0xF6,
/// which selects that interface.
constexpr uint8_t panel_setup[] = {
    0xCA, 3, 0xC3, 0x08, 0x50,
    0xCF, 3, 0x00, 0xC1, 0x30,
    0xED, 4, 0x64, 0x03, 0x12, 0x81,
    0xE8, 3, 0x85, 0x00, 0x78,
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xF7, 1, 0x20,
    0xEA, 2, 0x00, 0x00,
    0xB1, 2, 0x00, 0x1B,
    0xB6, 2, 0x0A, 0xA2,
    0xC0, 1, 0x10,
    0xC1, 1, 0x10,
    0xC5, 2, 0x45, 0x15,
    0xC7, 1, 0x90,
    0x36, 1, 0xC8,
    0xF2, 1, 0x00,
    0xB0, 1, 0xC2,
    0xB6, 4, 0x0A, 0xA7, 0x27, 0x04,
    0x2A, 4, 0x00, 0x00, 0x00, 0xEF,
    0x2B, 4, 0x00, 0x00, 0x01, 0x3F,
    0xF6, 3, 0x01, 0x00, 0x06,
    0x2C, 0,
};

constexpr uint8_t panel_gamma[] = {
    0x26, 1, 0x01,
    0xE0, 15, 0x0F, 0x29, 0x24, 0x0C, 0x0E, 0x09, 0x4E, 0x78, 0x3C, 0x09, 0x13, 0x05, 0x17,
    0x11, 0x00,
    0xE1, 15, 0x00, 0x16, 0x1B, 0x04, 0x11, 0x07, 0x31, 0x33, 0x42, 0x05, 0x0C, 0x0A, 0x28,
    0x2F, 0x0F,
};

/// Walk one of those tables. Answers how many BYTES went out, which is
/// what a program can say about a device that cannot answer.
uint16_t panel_sequence(const uint8_t* table, uint16_t length) {
    uint16_t sent = 0;
    uint16_t i = 0;
    while (i < length) {
        const uint8_t command = table[i];
        const uint8_t count = table[i + 1u];
        panel_write(command, &table[i + 2u], count);
        sent = static_cast<uint16_t>(sent + 1u + count);
        i = static_cast<uint16_t>(i + 2u + count);
    }
    return sent;
}

/// The whole of it, with the two waits the sequence asks for.
uint16_t panel_init() {
    panel_bus();
    uint16_t sent = panel_sequence(panel_setup, sizeof(panel_setup));
    wait_ms(200);
    sent = static_cast<uint16_t>(sent + panel_sequence(panel_gamma, sizeof(panel_gamma)));
    panel_write(0x11u, nullptr, 0);   // sleep out
    ++sent;
    wait_ms(200);
    panel_write(0x29u, nullptr, 0);   // display on
    ++sent;
    panel_write(0x2Cu, nullptr, 0);   // memory write, where the sequence ends
    ++sent;
    return sent;
}

// =============================================================================
// The controller, in the state the letters expect
// =============================================================================

/// One RGB565 layer covering the whole panel, out of the external
/// memory - the configuration every letter but j starts from.
LtdcLayerConfig one_layer() {
    LtdcLayerConfig c{};
    c.window = LtdcWindow{0, 0, panel_width, panel_height};
    c.format = LtdcPixelFormat::rgb565;
    c.framebuffer = sdram_base + fb16_offset;
    c.constant_alpha = 255;
    c.blend_source = LtdcBlend1::constant_alpha;
    c.blend_below = LtdcBlend2::one_minus_constant_alpha;
    return c;
}

/// The first line after the active area: the moment a program has a
/// whole vertical blanking in front of it, and the line every letter
/// that counts frames arms the event on.
constexpr uint16_t blanking_line =
    static_cast<uint16_t>(panel_timing.vsync + panel_timing.vbp + panel_timing.height);

/// Back to that state, whatever a letter left behind: layer 2 off, layer
/// 1 showing the 16-bit surface, the reload taken.
bool display_ready() {
    LtdcLayer<2>::disable();
    LtdcLayer<2>::blending(LtdcBlend1::pixel_alpha_x_constant,
                           LtdcBlend2::one_minus_pixel_alpha_x_constant);
    LtdcLayer<2>::default_colour(0);
    const bool ok = LtdcLayer<1>::configure(one_layer(), panel_timing);
    LtdcLayer<1>::enable();
    (void)Ltdc::line_interrupt(blanking_line);
    Ltdc::reload(LtdcReload::immediate);
    Ltdc::enable();
    return ok;
}

/// Wait for `frames` line events, which is the cheapest frame clock this
/// program has. False if the controller is not running.
bool wait_frames(uint32_t frames) {
    const uint32_t target = line_events + frames;
    const uint32_t deadline = millis() + frames * (frame_period_us / 1000u + 2u) + 50u;
    while (line_events < target) {
        if (static_cast<int32_t>(millis() - deadline) > 0) {
            return false;
        }
    }
    return true;
}

// =============================================================================
// a - the two blocks, and what they held at reset
// =============================================================================

void ta_blocks() {
    print(serial, "  this part: LTDC ", Ltdc::layers, " layers, DMA2D ",
          dma2d_present() ? "present" : "absent", ", PLLSAI R output ",
          pllsai_has_r() ? "yes" : "no", crlf);
    bench.verdict("the reserve finds a display controller of two layers, an accelerator and a "
                  "third PLL with the R output a pixel clock comes off",
                  ltdc_present() && Ltdc::layers == 2u && dma2d_present() && pllsai_has_r() &&
                      pllsai_has_lcd_divider());

    print(serial, "  vectors: LTDC ", static_cast<int32_t>(Ltdc::irq_line), ", LTDC error ",
          static_cast<int32_t>(Ltdc::error_irq_line), ", DMA2D ",
          static_cast<int32_t>(Dma2d::irq_line), crlf);
    bench.verdict("the controller has TWO vectors of its own and the accelerator a third",
                  Ltdc::irq_line != Ltdc::error_irq_line &&
                      Dma2d::irq_line != Ltdc::irq_line &&
                      Dma2d::irq_line != Ltdc::error_irq_line);

    print(serial, "  gates at reset: LTDC ", boot.ltdc_gate ? "OPEN" : "closed", ", DMA2D ",
          boot.dma2d_gate ? "OPEN" : "closed", ", PLLSAI ", boot.pllsai_on ? "ON" : "off", crlf);
    bench.verdict("both gates are closed at reset and the third PLL is off, so nothing of the "
                  "display tier costs a program that does not use it",
                  !boot.ltdc_gate && !boot.dma2d_gate && !boot.pllsai_on);

    // THE THREE CLOCK DOMAINS OF 16.3.2 ARE VISIBLE FROM THE BUS. With
    // the APB2 gate open and no pixel clock at all, every register of
    // the pixel-clock domain reads ZERO whatever its reset value is,
    // while the ones on PCLK2 and on HCLK read theirs.
    const Registers& dark = boot.dark;
    const Registers& lit = boot.clocked;
    print(serial, "  with no pixel clock: GCR ", hex(dark.gcr), " CDSR ", hex(dark.cdsr),
          " L1CACR ", hex(dark.l1cacr), " L1BFCR ", hex(dark.l1bfcr), " | SRCR ", hex(dark.srcr),
          " IER ", hex(dark.ier), " L1CR ", hex(dark.l1cr), " L1CFBAR ", hex(dark.l1cfbar), crlf);
    print(serial, "  with the pixel clock: GCR ", hex(lit.gcr), " CDSR ", hex(lit.cdsr),
          " L1CACR ", hex(lit.l1cacr), " L1BFCR ", hex(lit.l1bfcr), " | SRCR ", hex(lit.srcr),
          " IER ", hex(lit.ier), " L1CR ", hex(lit.l1cr), " L1CFBAR ", hex(lit.l1cfbar), crlf);
    bench.verdict("with the block clocked by the APB alone, every register of the PIXEL CLOCK "
                  "domain reads zero whatever its reset value - and the read answers rather than "
                  "hanging on a stall that cannot end",
                  dark.gcr == 0u && dark.cdsr == 0u && dark.l1cacr == 0u && dark.l1bfcr == 0u &&
                      dark.l1pfcr == 0u && dark.l1dccr == 0u && dark.l2cacr == 0u &&
                      dark.l2bfcr == 0u);
    bench.verdict("... while the registers 16.3.2 puts on PCLK2 and on HCLK read their reset "
                  "value with no pixel clock at all: table 88 is a fact of the bus and not only "
                  "of the block",
                  dark.srcr == lit.srcr && dark.ier == lit.ier && dark.isr == lit.isr &&
                      dark.l1cr == lit.l1cr && dark.l1cfbar == lit.l1cfbar &&
                      dark.l1cfblr == lit.l1cfblr && dark.l1cfblnr == lit.l1cfblnr &&
                      dark.l2cr == lit.l2cr);

    print(serial, "  clocked: SSCR ", hex(lit.sscr), " BPCR ", hex(lit.bpcr), " AWCR ",
          hex(lit.awcr), " TWCR ", hex(lit.twcr), " BCCR ", hex(lit.bccr), " LIPCR ",
          hex(lit.lipcr), crlf);
    // 16.7.5: 0x2220 is the three dither widths at two bits each, which
    // are READ-ONLY, with the enable and every polarity clear.
    bench.verdict("GCR reads 0x00002220 out of reset: the controller disabled, every polarity "
                  "active low, and the three dither widths reading two bits each",
                  lit.gcr == 0x0000'2220u);
    bench.verdict("the four timing registers, the reload, the background, the two interrupt "
                  "registers and the line position are all zero out of reset",
                  lit.sscr == 0u && lit.bpcr == 0u && lit.awcr == 0u && lit.twcr == 0u &&
                      lit.srcr == 0u && lit.bccr == 0u && lit.ier == 0u && lit.isr == 0u &&
                      lit.lipcr == 0u);
    // 16.7.13: every signal reads ACTIVE with the controller stopped.
    bench.verdict("CDSR reads 0x0000000F: with the timing generator held, all four display "
                  "signals read active", lit.cdsr == 0x0000'000Fu);

    print(serial, "  layer 1: CR ", hex(lit.l1cr), " PFCR ", hex(lit.l1pfcr), " CACR ",
          hex(lit.l1cacr), " BFCR ", hex(lit.l1bfcr), " DCCR ", hex(lit.l1dccr), crlf);
    print(serial, "  layer 2: CR ", hex(lit.l2cr), " CACR ", hex(lit.l2cacr), " BFCR ",
          hex(lit.l2bfcr), crlf);
    // 16.7.19 and 16.7.21: a layer comes up opaque and blending "pixel
    // alpha times constant" against "one minus it", which is what keeps
    // a disabled layer invisible.
    bench.verdict("both layers come up disabled, fully opaque (CACR 0xFF) and blending 0x0607 - "
                  "the pair that makes a disabled layer show nothing",
                  lit.l1cr == 0u && lit.l2cr == 0u && lit.l1cacr == 0xFFu &&
                      lit.l2cacr == 0xFFu && lit.l1bfcr == 0x0607u && lit.l2bfcr == 0x0607u);

    print(serial, "  DMA2D: CR ", hex(boot.d2cr), " ISR ", hex(boot.d2isr), " OPFCCR ",
          hex(boot.d2opfccr), " AMTCR ", hex(boot.d2amtcr), crlf);
    bench.verdict("the accelerator comes up idle, with no interrupt armed, no flag standing and "
                  "its dead time off",
                  boot.d2cr == 0u && boot.d2isr == 0u && boot.d2amtcr == 0u &&
                      boot.d2opfccr == 0u);

    print(serial, "  the memory ", boot.sdram_up ? "up" : "FAILED", ", the pixel clock ",
          boot.clock_up ? "locked" : "FAILED", ", the panel ", boot.panel_up ? "set" : "FAILED",
          " (", boot.panel_bytes, " bytes over SPI5)", crlf);
    bench.verdict("the boot brought up the external memory, locked the pixel clock and walked "
                  "the panel's sequence",
                  boot.sdram_up && boot.clock_up && boot.panel_up);
}

// =============================================================================
// b - the pixel clock
// =============================================================================

void tb_pixel_clock() {
    print(serial, "  PLLSAI: N ", pixel_pll.pll.n, " R ", pixel_pll.pll.r, " LCD divider ",
          static_cast<uint32_t>(pixel_pll.divider), crlf);
    const PllSaiConfig held = Rcc::pllsai_config();
    print(serial, "  the registers hold N ", held.n, " R ", held.r, " divider ",
          static_cast<uint32_t>(Rcc::lcd_clock_divider()), ", the PLL ",
          Rcc::pllsai_ready() ? "locked" : "UNLOCKED", crlf);
    bench.verdict("the triple the arithmetic solved is the triple the registers hold, and the "
                  "PLL is locked on it",
                  held.n == pixel_pll.pll.n && held.r == pixel_pll.pll.r &&
                      Rcc::lcd_clock_divider() == pixel_pll.divider && Rcc::pllsai_ready());

    const uint32_t computed = Ltdc::pixel_clock(8'000'000u);
    print(serial, "  LCD_CLK from the registers: ", computed / 1000u, " kHz", crlf);
    bench.verdict("the pixel clock the registers make is the one the panel was described with",
                  computed == pixel_hz);

    print(serial, "  the frame: ", ltdc_total_width(panel_timing), " x ",
          ltdc_total_height(panel_timing), " = ", frame_pixels, " pixel clocks, so ",
          frame_rate_mhz / 1000u, ".", (frame_rate_mhz % 1000u) / 100u, " Hz and ",
          frame_period_us, " us a frame", crlf);

    // The line event is the frame clock: how long 128 of them really
    // take, against the kernel's millisecond.
    (void)display_ready();
    Ltdc::clear(ltdc_event_mask(LtdcEvent::line));
    Ltdc::interrupt(LtdcEvent::line, true);
    Ltdc::enable_interrupt();
    line_events = 0;
    const bool armed = wait_frames(2);
    const uint32_t t0 = millis();
    const uint32_t n0 = line_events;
    const bool ran = wait_frames(128);
    const uint32_t elapsed = millis() - t0;
    const uint32_t frames = line_events - n0;
    Ltdc::interrupt(LtdcEvent::line, false);
    Ltdc::disable_interrupt();

    const uint32_t measured_us = (frames != 0u) ? (elapsed * 1000u) / frames : 0u;
    const uint32_t measured_hz_mhz = (measured_us != 0u) ? 1'000'000'000u / measured_us : 0u;
    print(serial, "  ", frames, " frames in ", elapsed, " ms: ", measured_us,
          " us a frame, ", measured_hz_mhz / 1000u, ".", (measured_hz_mhz % 1000u) / 100u,
          " Hz", crlf);
    const uint32_t derived_hz =
        (measured_us != 0u)
            ? static_cast<uint32_t>((static_cast<uint64_t>(frame_pixels) * 1'000'000u) /
                                    measured_us)
            : 0u;
    print(serial, "  the pixel clock that implies: ", derived_hz / 1000u, " kHz against the ",
          pixel_hz / 1000u, " kHz asked for", crlf);
    const uint32_t error_permille =
        (measured_us > frame_period_us)
            ? ((measured_us - frame_period_us) * 1000u) / frame_period_us
            : ((frame_period_us - measured_us) * 1000u) / frame_period_us;
    print(serial, "  the difference is ", error_permille, " per mille", crlf);
    bench.verdict("the frame period measured against SysTick is the one the timing and the PLL's "
                  "own ratio predict, inside a per cent",
                  armed && ran && error_permille <= 10u);
}

// =============================================================================
// c - what the drivers refuse
// =============================================================================

void tc_refusals() {
    const uint32_t sscr_before = Ltdc::regs().SSCR;
    bench.verdict("a timing with a zero synchronisation pulse is refused: every field of the "
                  "four registers holds one LESS than a count",
                  !Ltdc::timing(LtdcTiming{.hsync = 0, .hbp = 1, .width = 240, .hfp = 1,
                                           .vsync = 1, .vbp = 1, .height = 320, .vfp = 1}));
    bench.verdict("an active area past 1024x768 is refused (16.4.1)",
                  !Ltdc::timing(LtdcTiming{.hsync = 1, .hbp = 1, .width = 1025, .hfp = 1,
                                           .vsync = 1, .vbp = 1, .height = 320, .vfp = 1}));
    bench.verdict("... and a refused timing writes nothing", Ltdc::regs().SSCR == sscr_before);

    bench.verdict("a line interrupt past LIPOS's eleven bits is refused",
                  !Ltdc::line_interrupt(2048));

    const uint32_t whpcr_before = LtdcLayer<2>::regs().WHPCR;
    LtdcLayerConfig wide = one_layer();
    wide.window = LtdcWindow{1, 0, panel_width, panel_height};
    bench.verdict("a layer window that leaves the active area is refused",
                  !LtdcLayer<2>::configure(wide, panel_timing));
    LtdcLayerConfig keyed = one_layer();
    keyed.clut = true;
    bench.verdict("a colour table asked for on a format that has no index is refused",
                  !LtdcLayer<2>::configure(keyed, panel_timing));
    LtdcLayerConfig narrow = one_layer();
    narrow.pitch = 100;
    bench.verdict("a pitch narrower than the line it is the pitch of is refused",
                  !LtdcLayer<2>::configure(narrow, panel_timing));
    bench.verdict("... and none of the three wrote a layer register",
                  LtdcLayer<2>::regs().WHPCR == whpcr_before);

    bench.verdict("a line length past CFBLL's thirteen bits is refused",
                  !LtdcLayer<2>::buffer_line(8190, 8190));
    bench.verdict("a line count past CFBLNBR's eleven bits is refused",
                  !LtdcLayer<2>::buffer_lines(2048));

    // The accelerator's own refusals, over its registers.
    const uint32_t omar_before = Dma2d::regs().OMAR;
    constexpr Dma2dOutput out{
        .address = sdram_base + scratch_offset, .format = Dma2dOutputColor::rgb565};
    bench.verdict("a rectangle of no pixels is refused",
                  !Dma2d::fill(out, 0xFF00'0000UL, Dma2dArea{.pixels = 0, .lines = 8}));
    bench.verdict("a rectangle past NLR.PL's fourteen bits is refused",
                  !Dma2d::fill(out, 0xFF00'0000UL, Dma2dArea{.pixels = 16384, .lines = 1}));
    bench.verdict("an output address not aligned to its format is refused (11.5.4)",
                  !Dma2d::fill(Dma2dOutput{.address = sdram_base + scratch_offset + 1u,
                                           .format = Dma2dOutputColor::argb8888},
                               0xFF00'0000UL, Dma2dArea{.pixels = 8, .lines = 8}));
    bench.verdict("... and a refused fill writes no register", Dma2d::regs().OMAR == omar_before);

    constexpr Dma2dSource narrow_source{.address = 0x2000'0000u, .format = Dma2dColor::rgb565};
    bench.verdict("a plain memory-to-memory copy between two formats of different WIDTH is "
                  "refused: that mode does not convert, it moves bytes",
                  !Dma2d::copy(narrow_source,
                               Dma2dOutput{.address = sdram_base + scratch_offset,
                                           .format = Dma2dOutputColor::argb8888},
                               Dma2dArea{.pixels = 8, .lines = 8}));
    bench.verdict("... while the same pair through the pixel format converter is accepted",
                  Dma2d::convert(narrow_source,
                                 Dma2dOutput{.address = sdram_base + scratch_offset,
                                             .format = Dma2dOutputColor::argb8888},
                                 Dma2dArea{.pixels = 8, .lines = 8}));
    (void)Dma2d::wait();
    bench.verdict("a source with a 4-bit format and an ODD pixel count is refused (11.3.11)",
                  !Dma2d::convert(Dma2dSource{.address = 0x2000'0000u, .format = Dma2dColor::l4},
                                  out, Dma2dArea{.pixels = 15, .lines = 8}));

    (void)display_ready();
}

// =============================================================================
// d - the panel, and the one answer it can give
// =============================================================================

void td_panel() {
    print(serial, "  the sequence: ", boot.panel_bytes,
          " bytes out over SPI5 at ", Host::sck_hz(SpiClock::div32) / 1000u, " kHz", crlf);
    bench.verdict("the panel's whole sequence went out - the transaction shape this board's SPI "
                  "chapter measured against the other device on the same bus",
                  boot.panel_up && boot.panel_bytes > 100u);
    print(serial, "  the controller has no read path here: this board straps its interface so "
                  "that replies leave on a pad the MCU is not wired to (the SPI suite measured "
                  "that), so nothing it holds can be read back", crlf);

    // The tearing-effect output is the one thing the panel can say back:
    // command 0x35 turns it on, and if the device took the sequence it
    // starts driving PD11 once a frame.
    LcdTe::input(PinPull::down);
    const uint8_t te_on[1] = {0x00};
    panel_write(0x35u, te_on, 1);
    wait_ms(50);

    uint32_t edges = 0;
    bool previous = LcdTe::read();
    const uint32_t start = millis();
    while (millis() - start < 500u) {
        const bool now = LcdTe::read();
        if (now != previous) {
            ++edges;
            previous = now;
        }
    }
    panel_write(0x34u, nullptr, 0);   // and off again
    panel_bus();

    const uint32_t expected = (500u * 1000u) / frame_period_us;
    print(serial, "  the tearing-effect pad saw ", edges, " edges in 500 ms, against ", expected,
          " frames", crlf);
    bench.verdict("the panel answers a command with a signal: its tearing-effect output toggles "
                  "once a frame after 0x35, which says the sequence reached the device AND that "
                  "its frame timing is locked to the controller's",
                  edges >= expected && edges <= 3u * expected);

    (void)display_ready();
}

// =============================================================================
// e - the timing registers
// =============================================================================

void te_timing() {
    bench.verdict("the panel's timing is accepted", Ltdc::timing(panel_timing));
    const LTDC_TypeDef& r = Ltdc::regs();
    print(serial, "  SSCR ", hex(r.SSCR), " BPCR ", hex(r.BPCR), " AWCR ", hex(r.AWCR), " TWCR ",
          hex(r.TWCR), crlf);
    // 10/20/240/10 and 2/2/320/4, each field one less than the
    // accumulated count.
    bench.verdict("the four registers hold the ACCUMULATED numbers of 16.4.1, each one less than "
                  "its count: 9/1, 29/3, 269/323, 279/327",
                  r.SSCR == 0x0009'0001u && r.BPCR == 0x001D'0003u &&
                      r.AWCR == 0x010D'0143u && r.TWCR == 0x0117'0147u);

    const LtdcTiming back = Ltdc::timing();
    print(serial, "  read back: h ", back.hsync, "/", back.hbp, "/", back.width, "/", back.hfp,
          "  v ", back.vsync, "/", back.vbp, "/", back.height, "/", back.vfp, crlf);
    bench.verdict("the panel's own eight numbers come back out of the registers, the "
                  "accumulation undone",
                  back.hsync == panel_timing.hsync && back.hbp == panel_timing.hbp &&
                      back.width == panel_timing.width && back.hfp == panel_timing.hfp &&
                      back.vsync == panel_timing.vsync && back.vbp == panel_timing.vbp &&
                      back.height == panel_timing.height && back.vfp == panel_timing.vfp);
    bench.verdict("and so do the four polarities",
                  back.hsync_polarity == LtdcPolarity::active_low &&
                      back.vsync_polarity == LtdcPolarity::active_low &&
                      back.de_polarity == LtdcPolarity::active_low &&
                      back.clock_edge == LtdcClockEdge::direct);

    // The chapter's own worked example, on the silicon - and the one
    // place where 16.4.1's example does not obey 16.4.1's rule.
    constexpr LtdcTiming vga{.hsync = 8, .hbp = 7, .width = 640, .hfp = 5,
                             .vsync = 4, .vbp = 2, .height = 480, .vfp = 1};
    Ltdc::disable();
    bench.verdict("16.4.1's example timing is accepted", Ltdc::timing(vga));
    print(serial, "  its registers: SSCR ", hex(r.SSCR), " BPCR ", hex(r.BPCR), " AWCR ",
          hex(r.AWCR), " TWCR ", hex(r.TWCR), crlf);
    bench.verdict("the three registers whose example words 16.4.1 gets right are exactly those: "
                  "0x00070003, 0x000E0005 and 0x028E01E5",
                  r.SSCR == 0x0007'0003u && r.BPCR == 0x000E'0005u && r.AWCR == 0x028E'01E5u);
    // The rule says every field holds one LESS than its count, and the
    // example's own AAW and AAH obey it - but the two TOTAL fields it
    // prints (0x294 and 0x1E7) are the plain sums. The driver follows
    // the rule, and letter b's measured frame period is what says the
    // rule is right: at the example's arithmetic a frame would be a
    // line and a pixel longer than the one SysTick counted.
    print(serial, "  16.4.1's example prints TOTALW 0x294 and TOTALH 0x1E7, which are the plain "
                  "SUMS; its own rule - and its own AAW and AAH - subtract one, and the driver "
                  "does too: ", hex(r.TWCR), crlf);
    bench.verdict("the driver follows 16.4.1's RULE for the two total fields and not its "
                  "example's arithmetic, which prints the sums without the minus one",
                  r.TWCR == 0x0293'01E6u);

    print(serial, "  the polarities the other way round, then back", crlf);
    LtdcTiming flipped = panel_timing;
    flipped.hsync_polarity = LtdcPolarity::active_high;
    flipped.vsync_polarity = LtdcPolarity::active_high;
    flipped.de_polarity = LtdcPolarity::active_high;
    flipped.clock_edge = LtdcClockEdge::inverted;
    (void)Ltdc::timing(flipped);
    const uint32_t gcr_high = Ltdc::regs().GCR;
    (void)Ltdc::timing(panel_timing);
    print(serial, "  GCR with every polarity high ", hex(gcr_high), ", back low ",
          hex(Ltdc::regs().GCR), crlf);
    bench.verdict("the four polarity bits are GCR's top four and nothing else of the register "
                  "moves with them",
                  (gcr_high & 0xF000'0000u) == 0xF000'0000u &&
                      (gcr_high & 0x0FFF'FFFFu) == (Ltdc::regs().GCR & 0x0FFF'FFFFu) &&
                      (Ltdc::regs().GCR & 0xF000'0000u) == 0u);

    // 16.7.5: the three dither widths are read-only and this silicon
    // says two bits a channel, which is what 16.4.1 promises.
    print(serial, "  dither widths: R ", Ltdc::dither_red_width(), " G ",
          Ltdc::dither_green_width(), " B ", Ltdc::dither_blue_width(), crlf);
    bench.verdict("the three dither widths read two bits each, the width 16.4.1 states",
                  Ltdc::dither_red_width() == 2u && Ltdc::dither_green_width() == 2u &&
                      Ltdc::dither_blue_width() == 2u);
    Ltdc::dither(true);
    const bool on = Ltdc::dither();
    Ltdc::dither(false);
    bench.verdict("the dither enable is a bit of its own and the widths do not move with it",
                  on && !Ltdc::dither() && Ltdc::dither_red_width() == 2u);

    (void)display_ready();
}

// =============================================================================
// f - the position counter
// =============================================================================

void tf_position() {
    (void)display_ready();
    const uint16_t total_w = ltdc_total_width(panel_timing);
    const uint16_t total_h = ltdc_total_height(panel_timing);

    // What one read of a pixel-clock-domain register costs: 16.3.2's
    // seven PCLK2 periods plus five LCD_CLK ones, which at 6 MHz is
    // most of it.
    const uint32_t before = cycle_stamp();
    for (uint32_t i = 0; i < 16u; ++i) {
        (void)Ltdc::position();
    }
    const uint32_t spent = val_delta(before, cycle_stamp());
    print(serial, "  sixteen reads of CPSR cost ", spent, " core cycles: ", spent / 16u,
          " a read, ", (spent / 16u) / cycles_per_us, ".",
          ((spent / 16u) % cycles_per_us) * 10u / cycles_per_us, " us", crlf);
    // Five LCD_CLK periods at 6 MHz is 833 ns, which is 150 core cycles
    // at 180 MHz: the stall of 16.3.2 dominates and this is a
    // measurement of it, not a threshold to judge.
    bench.verdict("a read of a pixel-clock-domain register costs the stall 16.3.2 describes and "
                  "not a bus cycle: more than a hundred core cycles apiece",
                  spent / 16u > 100u);

    uint16_t max_x = 0;
    uint16_t max_y = 0;
    uint16_t min_x = 0xFFFFu;
    uint16_t min_y = 0xFFFFu;
    uint32_t past_the_frame = 0;
    bool moved = false;
    LtdcPosition previous = Ltdc::position();
    const uint32_t start = millis();
    while (millis() - start < 200u) {
        const LtdcPosition p = Ltdc::position();
        if (p.x != previous.x || p.y != previous.y) {
            moved = true;
        }
        previous = p;
        if (p.x > max_x) {
            max_x = p.x;
        }
        if (p.y > max_y) {
            max_y = p.y;
        }
        if (p.x < min_x) {
            min_x = p.x;
        }
        if (p.y < min_y) {
            min_y = p.y;
        }
        if (p.x >= total_w || p.y >= total_h) {
            ++past_the_frame;
        }
    }
    print(serial, "  over 200 ms the counter covered x ", min_x, "..", max_x, " and y ", min_y,
          "..", max_y, " against a frame of ", total_w, " x ", total_h, crlf);
    bench.verdict("the position counter walks, and it walks the WHOLE frame - porches and "
                  "synchronisation included - and never leaves it",
                  moved && past_the_frame == 0u && max_y >= total_h - 8u && min_y <= 4u);

    // CDSR: the four signals, seen in both states over a frame.
    uint8_t seen_high = 0;
    uint8_t seen_low = 0;
    const uint32_t status_start = millis();
    while (millis() - status_start < 100u) {
        const LtdcDisplayStatus s = Ltdc::display_status();
        const uint8_t bits = static_cast<uint8_t>((s.hsync ? 1u : 0u) | (s.vsync ? 2u : 0u) |
                                                  (s.horizontal_data_enable ? 4u : 0u) |
                                                  (s.vertical_data_enable ? 8u : 0u));
        seen_high = static_cast<uint8_t>(seen_high | bits);
        seen_low = static_cast<uint8_t>(seen_low | (~bits & 0x0Fu));
    }
    print(serial, "  CDSR bits seen high ", hex(seen_high), ", seen low ", hex(seen_low), crlf);
    bench.verdict("all four display-status bits are seen in BOTH states while the controller "
                  "runs: the register is the live state of the four signals and not a latch",
                  seen_high == 0x0Fu && seen_low == 0x0Fu);
}

// =============================================================================
// g - the line event
// =============================================================================

void tg_line_event() {
    (void)display_ready();
    const uint16_t total_h = ltdc_total_height(panel_timing);
    const uint16_t position = blanking_line;
    bench.verdict("the line position is accepted", Ltdc::line_interrupt(position));
    bench.verdict("... and reads back", Ltdc::line_interrupt() == position);

    line_events = 0;
    line_event_y = 0;
    Ltdc::clear(ltdc_event_mask(LtdcEvent::line));
    Ltdc::interrupt(LtdcEvent::line, true);
    Ltdc::enable_interrupt();
    const bool ran = wait_frames(4);
    const uint32_t t0 = millis();
    const uint32_t n0 = line_events;
    const bool more = wait_frames(64);
    const uint32_t elapsed = millis() - t0;
    const uint32_t frames = line_events - n0;
    const uint16_t landed = line_event_y;
    Ltdc::interrupt(LtdcEvent::line, false);
    Ltdc::disable_interrupt();

    print(serial, "  ", frames, " events in ", elapsed, " ms, one every ",
          frames != 0u ? (elapsed * 1000u) / frames : 0u, " us against a frame of ",
          frame_period_us, " us", crlf);
    bench.verdict("the line event arrives ONCE A FRAME, at the frame rate the timing predicts",
                  ran && more && frames == 64u &&
                      elapsed * 1000u > frames * frame_period_us * 99u / 100u &&
                      elapsed * 1000u < frames * frame_period_us * 101u / 100u);

    print(serial, "  the handler read the position counter at y ", landed, ", the event asked "
          "for line ", position, " of ", total_h, crlf);
    // The handler runs a few microseconds after the line starts and the
    // counter has moved on within the line, so the LINE is the claim and
    // the pixel is a measurement.
    bench.verdict("the event lands on the line it was programmed for",
                  landed == position || landed == position + 1u);

    // The reload event, on the same vector.
    reload_events = 0;
    Ltdc::clear(ltdc_event_mask(LtdcEvent::register_reload));
    Ltdc::interrupt(LtdcEvent::register_reload, true);
    Ltdc::enable_interrupt();
    LtdcLayer<1>::constant_alpha(254);
    Ltdc::reload(LtdcReload::vertical_blanking);
    const uint32_t deadline = millis() + 200u;
    while (reload_events == 0u && static_cast<int32_t>(millis() - deadline) < 0) {
    }
    const uint32_t reloads = reload_events;
    Ltdc::interrupt(LtdcEvent::register_reload, false);
    Ltdc::disable_interrupt();
    LtdcLayer<1>::constant_alpha(255);
    Ltdc::reload(LtdcReload::immediate);
    print(serial, "  the register-reload event fired ", reloads, " time(s) for one request",
          crlf);
    bench.verdict("a vertical-blanking reload raises its own event, on the SAME vector as the "
                  "line one", reloads >= 1u);

    print(serial, "  the transfer-error and FIFO-underrun events are the OTHER vector's, and "
          "letters i and j are what watch them", crlf);
}

// =============================================================================
// h - the shadow registers
// =============================================================================

void th_shadow() {
    (void)display_ready();
    // 16.4.1: a read of a shadowed register returns the ACTIVE value
    // until the reload has been done.
    LtdcLayer<1>::constant_alpha(0x80);
    const uint8_t before_reload = LtdcLayer<1>::constant_alpha();
    Ltdc::reload(LtdcReload::immediate);
    const bool waited = Ltdc::wait_reload();
    const uint8_t after_reload = LtdcLayer<1>::constant_alpha();
    print(serial, "  CACR written 0x80: reads ", hex(before_reload), " before the reload and ",
          hex(after_reload), " after", crlf);
    bench.verdict("an immediate reload takes the shadow and the register then reads what was "
                  "written", waited && after_reload == 0x80u);
    bench.verdict("... and before it, the read returned the value that was ACTIVE and not the "
                  "one just written - 16.4.1's own sentence, on the silicon",
                  before_reload == 0xFFu);

    // The immediate reload clears its own bit at once; the
    // vertical-blanking one waits for the blanking.
    LtdcLayer<1>::constant_alpha(0xFF);
    Ltdc::reload(LtdcReload::immediate);
    const bool immediate_gone = !Ltdc::reload_pending();
    bench.verdict("the immediate reload's bit is already clear on the next read: the hardware "
                  "took it inside the store's own bus cycle", immediate_gone);

    // How long the wait is depends on where in the frame the store
    // landed, so it is measured in wall milliseconds - a whole frame is
    // fifteen of them - and the CLAIM is only that it is never longer
    // than one frame.
    LtdcLayer<1>::constant_alpha(0x40);
    const uint32_t t0 = millis();
    Ltdc::reload(LtdcReload::vertical_blanking);
    const bool pending_at_once = Ltdc::reload_pending();
    const bool took = Ltdc::wait_reload();
    const uint32_t spent_ms = millis() - t0;
    const uint8_t alpha_now = LtdcLayer<1>::constant_alpha();
    print(serial, "  the vertical-blanking reload took ", spent_ms, " ms of the ",
          frame_period_us / 1000u, " a frame lasts (pending straight after the store: ",
          pending_at_once ? "yes" : "no", ")", crlf);
    bench.verdict("a vertical-blanking reload stands pending until the blanking comes, and then "
                  "the hardware clears it, never later than the next frame",
                  took && alpha_now == 0x40u && spent_ms <= frame_period_us / 1000u + 1u);

    LtdcLayer<1>::constant_alpha(0xFF);
    Ltdc::reload(LtdcReload::immediate);

    // The CLUT is the one layer register that is NOT shadowed, and
    // nothing reads it back: 16.4.1 says so and the register is
    // write-only (16.7.25). That is a fact this program can only state.
    print(serial, "  the colour table is the one layer register that is not shadowed, and it is "
          "write-only: no reload carries it and no read returns it", crlf);
}

// =============================================================================
// i - one layer out of the external memory
// =============================================================================

/// Watch the FIFO underrun over `frames` frames and answer whether it
/// stood at the end. The flag is cleared first, so what comes back is
/// whether the fetch kept up over that span.
///
/// `armed` sets IER's own bit for the two error events while the span
/// runs, WITHOUT enabling their vector: on this family a pending bit is
/// not always a thing that exists with its interrupt masked (the EXTI
/// chapter measured exactly that), so the two cases are two
/// measurements and not one.
uint32_t underruns_over(uint32_t frames, bool armed = true) {
    Ltdc::interrupt(LtdcEvent::fifo_underrun, armed);
    Ltdc::interrupt(LtdcEvent::transfer_error, armed);
    Ltdc::clear(ltdc_error_events);
    Ltdc::interrupt(LtdcEvent::line, true);
    Ltdc::enable_interrupt();
    line_events = 0;
    (void)wait_frames(frames);
    Ltdc::interrupt(LtdcEvent::line, false);
    Ltdc::disable_interrupt();
    const uint32_t seen = Ltdc::flag(LtdcEvent::fifo_underrun) ? 1u : 0u;
    Ltdc::interrupt(LtdcEvent::fifo_underrun, false);
    Ltdc::interrupt(LtdcEvent::transfer_error, false);
    return seen;
}

void ti_one_layer() {
    (void)display_ready();
    fb16.fill(rgb565(0, 0, 64));

    constexpr uint32_t bytes_a_second =
        ltdc_fetch_bytes_per_second(pixel_hz, panel_timing, LtdcPixelFormat::rgb565);
    print(serial, "  one 16-bit layer of ", panel_width, "x", panel_height, " fetches ",
          bytes_a_second / 1000u, " kB a second out of the external memory", crlf);

    const uint32_t bad = underruns_over(100);
    print(serial, "  over a hundred frames the FIFO underrun flag stood ", bad, " time(s)", crlf);
    bench.verdict("one layer of 16-bit pixels out of the external memory never starves the "
                  "controller's FIFO", bad == 0u);
    bench.verdict("... and no transfer error was raised either",
                  !Ltdc::flag(LtdcEvent::transfer_error));

    // The other half of 16.4.2: a frame buffer described SHORTER than
    // the window raises the underrun on purpose, which is what proves
    // the flag works at all. The chapter names both settings - the
    // number of lines and the line length - so both are tried.
    (void)LtdcLayer<1>::buffer_lines(panel_height / 2u);
    Ltdc::reload(LtdcReload::immediate);
    const uint32_t short_frame = underruns_over(10);
    (void)LtdcLayer<1>::buffer_lines(panel_height);
    Ltdc::reload(LtdcReload::immediate);
    (void)underruns_over(4);

    (void)LtdcLayer<1>::buffer_line(panel_width, 2u * panel_width);
    Ltdc::reload(LtdcReload::immediate);
    const uint32_t short_line = underruns_over(10);
    const uint32_t short_line_masked = underruns_over(10, false);
    (void)LtdcLayer<1>::buffer_line(2u * panel_width, 2u * panel_width);
    Ltdc::reload(LtdcReload::immediate);
    print(serial, "  half the LINES described: underrun ", short_frame,
          "; half the LINE LENGTH: underrun ", short_line, " armed, ", short_line_masked,
          " with the interrupt masked", crlf);
    bench.verdict("a frame buffer described with fewer bytes than the window needs - fewer LINES "
                  "or fewer BYTES A LINE, 16.4.2 names both - raises the underrun, which is what "
                  "makes the flag above a measurement and not an unread register",
                  short_frame == 1u && short_line == 1u);

    // And the same starved fetch on the OTHER vector, which is the one
    // the underrun and the transfer error come out of.
    error_events = 0;
    (void)LtdcLayer<1>::buffer_line(panel_width, 2u * panel_width);
    Ltdc::reload(LtdcReload::immediate);
    Ltdc::clear(ltdc_error_events);
    Ltdc::interrupt(LtdcEvent::fifo_underrun, true);
    Ltdc::enable_error_interrupt();
    wait_ms(3u * frame_period_us / 1000u + 2u);
    Ltdc::disable_error_interrupt();
    Ltdc::interrupt(LtdcEvent::fifo_underrun, false);
    const uint32_t taken = error_events;
    (void)LtdcLayer<1>::buffer_line(2u * panel_width, 2u * panel_width);
    Ltdc::reload(LtdcReload::immediate);
    Ltdc::clear(ltdc_error_events);
    print(serial, "  with the ERROR vector armed the same starved fetch took ", taken,
          " interrupt(s) over three frames - one every ",
          taken != 0u ? (3u * frame_period_us * 1000u) / taken : 0u, " ns", crlf);
    bench.verdict("the underrun comes out of the controller's SECOND vector, and the ISR body "
                  "with the error mask is what takes it there",
                  taken > 0u);
    // AND IT IS A STORM. The event is raised per PIXEL the FIFO cannot
    // serve and not once a frame, so a program that arms that vector
    // over a starved layer spends its core in the handler. Watching the
    // flag is the cheap way to learn the same thing.
    bench.verdict("... and it is raised per PIXEL and not once a frame: hundreds of times a "
                  "line, which makes the ERROR vector a thing to arm while a picture is known "
                  "good and the FLAG the thing to watch while it is not",
                  taken > 1000u);
    // THE TRAP, and the same one this family's EXTI has: the flag is
    // not a thing that stands whether or not anybody asked for it.
    bench.verdict("THE FLAG EXISTS ONLY WHILE ITS ENABLE IS SET: the same starved fetch leaves "
                  "ISR clear with IER's own bit clear, so a program that polls a status register "
                  "it never armed polls a register that cannot answer",
                  short_line_masked == 0u);

    (void)display_ready();
    (void)underruns_over(4);
}

// =============================================================================
// j - the bandwidth
// =============================================================================

/// A fill of the scratch area, repeated, timed in wall milliseconds -
/// the accelerator's own rate, with whatever else is on the memory.
uint32_t fill_pixels_per_us(uint32_t repeats) {
    constexpr uint32_t pixels = 256u;
    constexpr uint32_t lines = 256u;
    const Dma2dOutput out{.address = sdram_base + scratch_offset,
                          .line_offset = 0,
                          .format = Dma2dOutputColor::argb8888};
    const Dma2dArea area{.pixels = pixels, .lines = lines};
    const uint32_t t0 = millis();
    uint32_t done = 0;
    for (uint32_t i = 0; i < repeats; ++i) {
        if (!Dma2d::fill(out, 0xFF20'4060UL, area)) {
            break;
        }
        if (!Dma2d::wait()) {
            break;
        }
        ++done;
    }
    const uint32_t elapsed = millis() - t0;
    if (elapsed == 0u || done == 0u) {
        return 0u;
    }
    return (done * pixels * lines) / (elapsed * 1000u);
}

/// One configuration measured: the fetch it costs, the underrun with
/// nothing else on the memory and with the accelerator filling it, and
/// the fill's own rate under that load. Every measurement is taken after
/// a few frames of settling, because the reload that installed the
/// configuration lands in the middle of a frame and that frame's fetch
/// is half of the old picture and half of the new one.
struct Loaded {
    uint32_t idle_underrun = 0;
    uint32_t loaded_underrun = 0;
    uint32_t fill_rate = 0;
};

Loaded measure_configuration() {
    Loaded r{};
    (void)underruns_over(4);
    r.idle_underrun = underruns_over(60);
    (void)underruns_over(2);
    r.fill_rate = fill_pixels_per_us(20);
    r.loaded_underrun = underruns_over(60);
    return r;
}

void tj_bandwidth() {
    (void)display_ready();
    Dma2d::init();

    constexpr uint32_t shallow_bytes =
        ltdc_fetch_bytes_per_second(pixel_hz, panel_timing, LtdcPixelFormat::rgb565);
    constexpr uint32_t deep_bytes =
        ltdc_fetch_bytes_per_second(pixel_hz, panel_timing, LtdcPixelFormat::argb8888);
    print(serial, "  the memory is 16 bits wide at 90 MHz; a layer's fetch is its active area "
          "times the frame rate, and the accelerator writes into what is left", crlf);

    // Configuration 1: one 16-bit layer.
    const Loaded shallow = measure_configuration();
    print(serial, "  one layer, 16 bpp (", shallow_bytes / 1000u, " kB/s fetched): underrun ",
          shallow.idle_underrun, " idle, ", shallow.loaded_underrun, " loaded; the fill ",
          shallow.fill_rate, " pixels a us = ", shallow.fill_rate * 4u, " MB/s, so ",
          shallow.fill_rate * 4u + shallow_bytes / 1'000'000u, " MB/s over the bus", crlf);

    // Configuration 2: one 32-bit layer.
    fb32a.fill(argb8888(255, 32, 32, 96));
    LtdcLayerConfig deep = one_layer();
    deep.format = LtdcPixelFormat::argb8888;
    deep.framebuffer = sdram_base + fb32a_offset;
    (void)LtdcLayer<1>::configure(deep, panel_timing);
    LtdcLayer<1>::enable();
    Ltdc::reload(LtdcReload::immediate);
    const Loaded one_deep = measure_configuration();
    print(serial, "  one layer, 32 bpp (", deep_bytes / 1000u, " kB/s fetched): underrun ",
          one_deep.idle_underrun, " idle, ", one_deep.loaded_underrun, " loaded; the fill ",
          one_deep.fill_rate, " pixels a us = ", one_deep.fill_rate * 4u, " MB/s, so ",
          one_deep.fill_rate * 4u + deep_bytes / 1'000'000u, " MB/s over the bus", crlf);

    // Configuration 3: two 32-bit layers, the top one half transparent.
    fb32b.fill(argb8888(128, 200, 40, 40));
    LtdcLayerConfig top = deep;
    top.framebuffer = sdram_base + fb32b_offset;
    top.constant_alpha = 128;
    top.blend_source = LtdcBlend1::pixel_alpha_x_constant;
    top.blend_below = LtdcBlend2::one_minus_pixel_alpha_x_constant;
    (void)LtdcLayer<2>::configure(top, panel_timing);
    LtdcLayer<2>::enable();
    Ltdc::reload(LtdcReload::immediate);
    const Loaded two_deep = measure_configuration();
    print(serial, "  two layers, 32 bpp (", 2u * deep_bytes / 1000u, " kB/s fetched): underrun ",
          two_deep.idle_underrun, " idle, ", two_deep.loaded_underrun, " loaded; the fill ",
          two_deep.fill_rate, " pixels a us = ", two_deep.fill_rate * 4u, " MB/s, so ",
          two_deep.fill_rate * 4u + 2u * deep_bytes / 1'000'000u, " MB/s over the bus", crlf);

    bench.verdict("NO CONFIGURATION STARVES THE CONTROLLER - one layer or two, sixteen bits a "
                  "pixel or thirty-two, with the accelerator filling the same memory or not",
                  shallow.idle_underrun == 0u && shallow.loaded_underrun == 0u &&
                      one_deep.idle_underrun == 0u && one_deep.loaded_underrun == 0u &&
                      two_deep.idle_underrun == 0u && two_deep.loaded_underrun == 0u);
    bench.verdict("the accelerator's own rate falls as the display's fetch rises: the two "
                  "masters share ONE memory and the display is served first",
                  shallow.fill_rate >= one_deep.fill_rate &&
                      one_deep.fill_rate >= two_deep.fill_rate);
    print(serial, "  the three totals above are what this board's memory really delivers; what "
          "each costs is the device's rate, the bus width and the refresh, all three of them "
          "this board's and not the family's", crlf);

    // The dead time is the one knob: what it does to the fill's rate.
    Dma2d::dead_time(32, true);
    const uint32_t throttled = fill_pixels_per_us(10);
    Dma2d::dead_time(0, false);
    print(serial, "  with 32 AHB cycles of dead time between accesses the fill drops to ",
          throttled, " pixels a us from ", two_deep.fill_rate, crlf);
    bench.verdict("the AHB dead time really throttles the accelerator - the one bandwidth knob "
                  "the chapter has", throttled < two_deep.fill_rate);

    (void)display_ready();
    (void)underruns_over(4);
}

// =============================================================================
// k - the accelerator's four modes, byte-exact
// =============================================================================

// The model buffers live in INTERNAL memory: the CPU reads them back
// word for word, which ES0206 2.3.5 says it may not safely do on the
// external one. They are VOLATILE because another bus master writes
// them - without that the compiler is entitled to answer a read out of
// its own last store, and the accelerator's address travels to it as a
// plain integer that nothing in the program dereferences.
constexpr uint16_t model_w = 16;
constexpr uint16_t model_h = 8;
volatile uint32_t source32[model_w * model_h];
volatile uint32_t source32b[model_w * model_h];
volatile uint16_t source16[model_w * model_h];
volatile uint32_t result32[model_w * model_h];
volatile uint16_t result16[model_w * model_h];
volatile uint8_t source8[model_w * model_h];

void tk_modes() {
    Dma2d::init();
    const Dma2dArea area{.pixels = model_w, .lines = model_h};

    // 1. Register to memory: one colour into a rectangle.
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        result32[i] = 0xDEAD'BEEFu;
    }
    const uint32_t colour = argb8888(0x80, 0x12, 0x34, 0x56);
    const bool filled =
        Dma2d::fill(Dma2dOutput{.address = reinterpret_cast<uint32_t>(result32),
                                .format = Dma2dOutputColor::argb8888},
                    colour, area) &&
        Dma2d::wait();
    uint32_t bad = 0;
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        if (result32[i] != colour) {
            ++bad;
        }
    }
    print(serial, "  fill: ", model_w, "x", model_h, " words of ", hex(colour), ", ", bad,
          " wrong", crlf);
    bench.verdict("register to memory writes the colour and nothing else, byte for byte",
                  filled && bad == 0u);

    // 2. Memory to memory: bytes moved, no conversion.
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        source32[i] = i * 0x0101'0101u + 0x1234'5678u;
        result32[i] = 0;
    }
    const bool copied =
        Dma2d::copy(Dma2dSource{.address = reinterpret_cast<uint32_t>(source32),
                                .format = Dma2dColor::argb8888},
                    Dma2dOutput{.address = reinterpret_cast<uint32_t>(result32),
                                .format = Dma2dOutputColor::argb8888},
                    area) &&
        Dma2d::wait();
    bad = 0;
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        if (result32[i] != source32[i]) {
            ++bad;
        }
    }
    print(serial, "  copy: ", bad, " words wrong", crlf);
    bench.verdict("memory to memory moves the rectangle unchanged", copied && bad == 0u);

    // 3. With the pixel format converter: RGB565 in, ARGB8888 out. The
    // model is 16.4.2's own rule - the channels expanded by replicating
    // their most significant bits - and the alpha filled in as opaque.
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        source16[i] = static_cast<uint16_t>(i * 0x0421u + 0x1234u);
        result32[i] = 0;
    }
    const bool converted =
        Dma2d::convert(Dma2dSource{.address = reinterpret_cast<uint32_t>(source16),
                                   .format = Dma2dColor::rgb565},
                       Dma2dOutput{.address = reinterpret_cast<uint32_t>(result32),
                                   .format = Dma2dOutputColor::argb8888},
                       area) &&
        Dma2d::wait();
    bad = 0;
    uint32_t first_got = 0;
    uint32_t first_want = 0;
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        const uint16_t p = source16[i];
        const uint8_t r5 = static_cast<uint8_t>((p >> 11) & 0x1Fu);
        const uint8_t g6 = static_cast<uint8_t>((p >> 5) & 0x3Fu);
        const uint8_t b5 = static_cast<uint8_t>(p & 0x1Fu);
        const uint32_t want = argb8888(0xFF, static_cast<uint8_t>((r5 << 3) | (r5 >> 2)),
                                       static_cast<uint8_t>((g6 << 2) | (g6 >> 4)),
                                       static_cast<uint8_t>((b5 << 3) | (b5 >> 2)));
        if (result32[i] != want) {
            if (bad == 0u) {
                first_got = result32[i];
                first_want = want;
            }
            ++bad;
        }
    }
    print(serial, "  RGB565 -> ARGB8888: ", bad, " wrong");
    if (bad != 0u) {
        print(serial, ", first ", hex(first_got), " against ", hex(first_want));
    }
    print(serial, crlf);
    bench.verdict("the pixel format converter expands each channel by replicating its most "
                  "significant bits into the least, and fills in an opaque alpha where the "
                  "source has none - 16.4.2's rule, byte-exact",
                  converted && bad == 0u);

    // 4. Blending: two known colours with known alphas, against
    // 11.3.11's formula.
    const uint32_t fg = argb8888(0x80, 0xC0, 0x40, 0x20);
    const uint32_t bg = argb8888(0xFF, 0x10, 0x80, 0xF0);
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        source32[i] = fg;
        source32b[i] = bg;
        result32[i] = 0;
    }
    const bool blended =
        Dma2d::blend(Dma2dSource{.address = reinterpret_cast<uint32_t>(source32),
                                 .format = Dma2dColor::argb8888},
                     Dma2dSource{.address = reinterpret_cast<uint32_t>(source32b),
                                 .format = Dma2dColor::argb8888},
                     Dma2dOutput{.address = reinterpret_cast<uint32_t>(result32),
                                 .format = Dma2dOutputColor::argb8888},
                     area) &&
        Dma2d::wait();
    const uint8_t want_a = dma2d_blend_alpha(0x80, 0xFF);
    const uint32_t want =
        argb8888(want_a, dma2d_blend_channel(0x80, 0xFF, 0xC0, 0x10),
                 dma2d_blend_channel(0x80, 0xFF, 0x40, 0x80),
                 dma2d_blend_channel(0x80, 0xFF, 0x20, 0xF0));
    bad = 0;
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        if (result32[i] != want) {
            if (bad == 0u) {
                first_got = result32[i];
            }
            ++bad;
        }
    }
    print(serial, "  blend ", hex(fg), " over ", hex(bg), " -> ", hex(first_got != 0u ? first_got
                                                                                     : result32[0]),
          ", the formula says ", hex(want), ", ", bad, " wrong", crlf);
    bench.verdict("the blender is 11.3.11's formula exactly, divisions rounded down and all",
                  blended && bad == 0u);

    // The alpha modes of the converter, on the same source.
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        source32[i] = argb8888(0x80, 0x10, 0x20, 0x30);
        result32[i] = 0;
    }
    const bool replaced =
        Dma2d::convert(Dma2dSource{.address = reinterpret_cast<uint32_t>(source32),
                                   .format = Dma2dColor::argb8888,
                                   .alpha_mode = Dma2dAlpha::replace,
                                   .alpha = 0x40},
                       Dma2dOutput{.address = reinterpret_cast<uint32_t>(result32),
                                   .format = Dma2dOutputColor::argb8888},
                       area) &&
        Dma2d::wait();
    const uint32_t after_replace = result32[0];
    const bool multiplied =
        Dma2d::convert(Dma2dSource{.address = reinterpret_cast<uint32_t>(source32),
                                   .format = Dma2dColor::argb8888,
                                   .alpha_mode = Dma2dAlpha::multiply,
                                   .alpha = 0x80},
                       Dma2dOutput{.address = reinterpret_cast<uint32_t>(result32),
                                   .format = Dma2dOutputColor::argb8888},
                       area) &&
        Dma2d::wait();
    const uint32_t after_multiply = result32[0];
    print(serial, "  alpha replaced -> ", hex(after_replace), ", multiplied -> ",
          hex(after_multiply), crlf);
    bench.verdict("the converter's alpha modes are the chapter's: replaced by the field, and "
                  "multiplied by it over 255",
                  replaced && multiplied && (after_replace >> 24) == 0x40u &&
                      (after_multiply >> 24) == dma2d_source_alpha(Dma2dAlpha::multiply, 0x80,
                                                                   0x80) &&
                      (after_replace & 0x00FF'FFFFu) == 0x0010'2030u);

    // The output formats: the same word packed five ways.
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        result16[i] = 0;
    }
    const bool packed = Dma2d::fill(Dma2dOutput{.address = reinterpret_cast<uint32_t>(result16),
                                                .format = Dma2dOutputColor::rgb565},
                                    argb8888(0xFF, 0xFF, 0x80, 0x00), area) &&
                        Dma2d::wait();
    print(serial, "  a fill in RGB565 wrote ", hex(result16[0]), ", the packer says ",
          hex(rgb565(0xFF, 0x80, 0x00)), crlf);
    bench.verdict("an output in a narrower format is packed as the chapter's table says",
                  packed && result16[0] == rgb565(0xFF, 0x80, 0x00));

    // The configuration error: an address the format is not aligned to,
    // written past the driver's own refusal through the resource.
    Dma2d::clear(0x3Fu);
    Dma2d::regs().OMAR = reinterpret_cast<uint32_t>(result32) + 1u;
    Dma2d::regs().OPFCCR = static_cast<uint32_t>(Dma2dOutputColor::argb8888);
    Dma2d::regs().OOR = 0;
    Dma2d::regs().FGMAR = reinterpret_cast<uint32_t>(source32);
    Dma2d::regs().FGOR = 0;
    Dma2d::regs().FGPFCCR = static_cast<uint32_t>(Dma2dColor::argb8888);
    (void)Dma2d::start(Dma2dMode::memory_to_memory_pfc, area);
    (void)Dma2d::wait(100'000u);
    const bool config_error = Dma2d::flag(Dma2dEvent::configuration_error);
    const bool no_transfer = !Dma2d::flag(Dma2dEvent::transfer_complete);
    Dma2d::clear(0x3Fu);
    print(serial, "  a misaligned destination started: configuration error ",
          config_error ? "raised" : "NOT raised", ", transfer complete ",
          no_transfer ? "not raised" : "raised", crlf);
    bench.verdict("the silicon checks the configuration itself and answers a wrong one with the "
                  "configuration-error flag and no transfer at all - which is why the driver's "
                  "own refusals sit in front of it",
                  config_error && no_transfer);
}

// =============================================================================
// l - the colour tables
// =============================================================================

/// The table the engine loads out of memory - read by a master of its
/// own, so volatile like the model buffers above.
volatile uint32_t clut_source[256];

void tl_clut() {
    Dma2d::init();
    const Dma2dArea area{.pixels = model_w, .lines = model_h};

    // A sixteen-entry table, written through the CPU into the
    // converter's own memory, then an L8 source converted through it.
    for (uint32_t i = 0; i < 16u; ++i) {
        Dma2d::clut_entry(static_cast<uint8_t>(i),
                          argb8888(0xFF, static_cast<uint8_t>(i * 16u),
                                   static_cast<uint8_t>(255u - i * 16u),
                                   static_cast<uint8_t>(i * 8u)));
    }
    uint32_t mismatched = 0;
    for (uint32_t i = 0; i < 16u; ++i) {
        const uint32_t want = argb8888(0xFF, static_cast<uint8_t>(i * 16u),
                                       static_cast<uint8_t>(255u - i * 16u),
                                       static_cast<uint8_t>(i * 8u));
        if (Dma2d::clut_word(static_cast<uint8_t>(i)) != want) {
            ++mismatched;
        }
    }
    print(serial, "  sixteen entries written by the CPU, ", mismatched, " read back wrong", crlf);
    bench.verdict("the converter's colour table is ordinary memory to the CPU while the engine "
                  "is idle", mismatched == 0u);

    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        source8[i] = static_cast<uint8_t>(i & 0x0Fu);
        result32[i] = 0;
    }
    const bool indexed =
        Dma2d::convert(Dma2dSource{.address = reinterpret_cast<uint32_t>(source8),
                                   .format = Dma2dColor::l8},
                       Dma2dOutput{.address = reinterpret_cast<uint32_t>(result32),
                                   .format = Dma2dOutputColor::argb8888},
                       area) &&
        Dma2d::wait();
    uint32_t bad = 0;
    for (uint32_t i = 0; i < model_w * model_h; ++i) {
        const uint32_t index = i & 0x0Fu;
        const uint32_t want = argb8888(0xFF, static_cast<uint8_t>(index * 16u),
                                       static_cast<uint8_t>(255u - index * 16u),
                                       static_cast<uint8_t>(index * 8u));
        if (result32[i] != want) {
            ++bad;
        }
    }
    print(serial, "  an L8 rectangle through it: ", bad, " wrong", crlf);
    bench.verdict("an indexed source is looked up in that table, entry for entry",
                  indexed && bad == 0u);

    // And the same table loaded by the ENGINE out of memory, which is
    // the other of 11.3.11's two ways.
    for (uint32_t i = 0; i < 256u; ++i) {
        clut_source[i] = argb8888(0xFF, static_cast<uint8_t>(255u - i),
                                  static_cast<uint8_t>(i), static_cast<uint8_t>(i * 3u));
    }
    for (uint32_t i = 0; i < 16u; ++i) {
        Dma2d::clut_entry(static_cast<uint8_t>(i), 0);
    }
    const Dma2dSource with_table{.address = reinterpret_cast<uint32_t>(source8),
                                 .format = Dma2dColor::l8,
                                 .clut_address = reinterpret_cast<uint32_t>(clut_source),
                                 .clut_entries = 256,
                                 .clut_format = Dma2dClutColor::argb8888};
    Dma2d::clear(0x3Fu);
    const bool started = Dma2d::load_clut(with_table);
    const bool loaded = Dma2d::wait_clut();
    const bool announced = Dma2d::flag(Dma2dEvent::clut_transfer_complete);
    Dma2d::clear(0x3Fu);
    uint32_t wrong = 0;
    for (uint32_t i = 0; i < 256u; ++i) {
        if (Dma2d::clut_word(static_cast<uint8_t>(i)) != clut_source[i]) {
            ++wrong;
        }
    }
    print(serial, "  256 entries loaded by the engine: started ", started ? "yes" : "no",
          ", complete flag ", announced ? "raised" : "not raised", ", ", wrong,
          " read back wrong", crlf);
    bench.verdict("the engine loads a whole colour table out of memory by itself and says so "
                  "with its own flag", started && loaded && announced && wrong == 0u);

    // The controller's own colour tables are WRITE-ONLY: an L8 layer is
    // shown through one and nothing but the panel can say it worked.
    (void)display_ready();
    LtdcLayerConfig indexed_layer = one_layer();
    indexed_layer.format = LtdcPixelFormat::l8;
    indexed_layer.framebuffer = sdram_base + fb8_offset;
    indexed_layer.clut = true;
    const bool accepted = LtdcLayer<1>::configure(indexed_layer, panel_timing);
    for (uint32_t i = 0; i < 256u; ++i) {
        LtdcLayer<1>::clut_entry(static_cast<uint8_t>(i), static_cast<uint8_t>(i),
                                 static_cast<uint8_t>(255u - i), static_cast<uint8_t>(i / 2u));
    }
    volatile uint8_t* indexed_pixels = Sdram::at<uint8_t>(fb8_offset);
    for (uint32_t y = 0; y < panel_height; ++y) {
        for (uint32_t x = 0; x < panel_width; ++x) {
            indexed_pixels[y * panel_width + x] = static_cast<uint8_t>(x);
        }
    }
    LtdcLayer<1>::enable();
    Ltdc::reload(LtdcReload::immediate);
    const uint32_t bad_frames = underruns_over(30);
    print(serial, "  an L8 layer through the controller's own table: accepted ",
          accepted ? "yes" : "no", ", underrun ", bad_frames, " over thirty frames", crlf);
    bench.verdict("an indexed layer of one byte a pixel is fetched without starving - the table "
                  "itself is write-only, so what it PAINTS needs eyes",
                  accepted && bad_frames == 0u);

    (void)display_ready();
    (void)underruns_over(4);
}

// =============================================================================
// m - the layer geometry and the blending registers
// =============================================================================

void tm_layer() {
    (void)display_ready();
    const uint16_t ahbp = Ltdc::horizontal_back_porch();
    const uint16_t avbp = Ltdc::vertical_back_porch();
    print(serial, "  the accumulated back porches: AHBP ", ahbp, ", AVBP ", avbp, crlf);
    bench.verdict("the block's own back porches are the panel's hsync + hbp - 1 and vsync + vbp "
                  "- 1, which is what a layer's position is counted from",
                  ahbp == panel_timing.hsync + panel_timing.hbp - 1u &&
                      avbp == panel_timing.vsync + panel_timing.vbp - 1u);

    // A known starting point, so that "the active value" below is one
    // this letter put there and not whatever the letter before left.
    const LtdcWindow previous{0, 0, panel_width, panel_height};
    LtdcLayer<2>::window(previous);
    Ltdc::reload(LtdcReload::immediate);
    const uint32_t whpcr_previous = LtdcLayer<2>::regs().WHPCR;

    const LtdcWindow asked{40, 60, 100, 120};
    LtdcLayer<2>::window(asked);
    // EVERY layer register but the colour table is shadowed (16.4.1),
    // LxCR and the frame buffer's three included - so a read before the
    // reload answers the ACTIVE value, which here is the window that
    // was in force a moment ago.
    const uint32_t whpcr_before = LtdcLayer<2>::regs().WHPCR;
    Ltdc::reload(LtdcReload::immediate);
    const uint32_t whpcr = LtdcLayer<2>::regs().WHPCR;
    const uint32_t wvpcr = LtdcLayer<2>::regs().WVPCR;
    print(serial, "  the window in force was ", hex(whpcr_previous), "; a new one at ", asked.x,
          ",", asked.y, " of ", asked.width, "x", asked.height, " reads ", hex(whpcr_before),
          " before the reload and ", hex(whpcr), " after; WVPCR ", hex(wvpcr), crlf);
    bench.verdict("a layer register read BEFORE its reload answers the ACTIVE value and not what "
                  "was written: the whole layer is shadowed, its geometry as much as its alpha",
                  whpcr_before == whpcr_previous && whpcr != whpcr_previous);

    // 16.7.15: the first visible pixel of a line is AHBP + 1.
    const uint16_t want_h0 = static_cast<uint16_t>(ahbp + 1u + asked.x);
    const uint16_t want_v0 = static_cast<uint16_t>(avbp + 1u + asked.y);
    bench.verdict("the registers hold the ACCUMULATED position 16.7.15 asks for - the back porch "
                  "plus one plus the window's own offset - and the stop is the last visible "
                  "pixel and not one past it",
                  (whpcr & 0x0FFFu) == want_h0 &&
                      ((whpcr >> 16) & 0x0FFFu) == want_h0 + asked.width - 1u &&
                      (wvpcr & 0x07FFu) == want_v0 &&
                      ((wvpcr >> 16) & 0x07FFu) == want_v0 + asked.height - 1u);
    const LtdcWindow back = LtdcLayer<2>::window();
    print(serial, "  read back: ", back.x, ",", back.y, " of ", back.width, "x", back.height,
          crlf);
    bench.verdict("... and the window comes back in the picture's own coordinates",
                  back.x == asked.x && back.y == asked.y && back.width == asked.width &&
                      back.height == asked.height);

    // THE RACE. A read issued straight after a RAW store into SRCR may
    // still answer the pre-reload value: the reload crosses into the
    // pixel-clock domain and the read is on its way there already. It
    // is a race the two clocks decide, so it is COUNTED and not judged
    // - eight rounds each way. The driver's own reload() spends one
    // access of that domain (a read of GCR, five LCD_CLK periods of
    // stall by 16.3.2) after the store, which is longer than the
    // crossing and is what closes it.
    uint32_t raw_stale = 0;
    uint32_t verb_stale = 0;
    for (uint32_t round = 0; round < 8u; ++round) {
        const LtdcWindow settled{static_cast<uint16_t>(10u + round), 20, 80, 90};
        const LtdcWindow moved{static_cast<uint16_t>(60u + round), 20, 80, 90};

        LtdcLayer<2>::window(settled);
        Ltdc::reload(LtdcReload::immediate);
        const uint32_t was = LtdcLayer<2>::regs().WHPCR;
        LtdcLayer<2>::window(moved);
        Ltdc::regs().SRCR = LTDC_SRCR_IMR;
        if (LtdcLayer<2>::regs().WHPCR == was) {
            ++raw_stale;
        }

        LtdcLayer<2>::window(settled);
        Ltdc::reload(LtdcReload::immediate);
        const uint32_t was_again = LtdcLayer<2>::regs().WHPCR;
        LtdcLayer<2>::window(moved);
        Ltdc::reload(LtdcReload::immediate);
        if (LtdcLayer<2>::regs().WHPCR == was_again) {
            ++verb_stale;
        }
    }
    print(serial, "  over eight rounds the first read after a RAW reload store was stale ",
          raw_stale, " time(s); after the driver's reload(), ", verb_stale, crlf);
    bench.verdict("the driver's reload() spends one access of the pixel-clock domain after the "
                  "store, and the caller's first read back is the reloaded value every time",
                  verb_stale == 0u);

    // The frame buffer's two length fields, with the chapter's "+ 3".
    (void)LtdcLayer<2>::buffer_line(ltdc_line_length(100, LtdcPixelFormat::rgb565) - 3u, 480);
    Ltdc::reload(LtdcReload::immediate);
    const uint32_t cfblr = LtdcLayer<2>::regs().CFBLR;
    print(serial, "  CFBLR for a 100-pixel line of 16-bit pixels at a 480-byte pitch: ",
          hex(cfblr), crlf);
    bench.verdict("the line length register holds the bytes PLUS THREE and the pitch beside it, "
                  "16.7.23's own arithmetic",
                  (cfblr & 0x1FFFu) == 203u && ((cfblr >> 16) & 0x1FFFu) == 480u);

    // Blending, keying, the default colour: written and read back.
    LtdcLayer<2>::constant_alpha(0x30);
    LtdcLayer<2>::blending(LtdcBlend1::constant_alpha, LtdcBlend2::one_minus_constant_alpha);
    LtdcLayer<2>::default_colour(argb8888(0x11, 0x22, 0x33, 0x44));
    LtdcLayer<2>::colour_key(rgb888(0xAA, 0xBB, 0xCC));
    LtdcLayer<2>::colour_keying(true);
    LtdcLayer<2>::clut(false);
    Ltdc::reload(LtdcReload::immediate);
    const uint32_t bfcr = LtdcLayer<2>::regs().BFCR;
    print(serial, "  CACR ", hex(LtdcLayer<2>::constant_alpha()), " BFCR ", hex(bfcr), " DCCR ",
          hex(LtdcLayer<2>::default_colour()), " CKCR ", hex(LtdcLayer<2>::colour_key()), crlf);
    bench.verdict("the constant alpha, the two blending factors, the default colour and the "
                  "colour key are all written and all read back",
                  LtdcLayer<2>::constant_alpha() == 0x30u && bfcr == 0x0405u &&
                      LtdcLayer<2>::default_colour() == 0x1122'3344u &&
                      LtdcLayer<2>::colour_key() == 0x00AA'BBCCu);
    bench.verdict("the two legal blending codes are 4 and 5 for the constant alpha pair, which "
                  "is what the register now holds", (bfcr & 0x0700u) == 0x0400u &&
                                                        (bfcr & 0x0007u) == 0x0005u);

    // The enable is shadowed like the rest: a layer turned on is not on
    // until the reload, which is exactly what makes a two-layer change
    // atomic.
    LtdcLayer<2>::enable();
    const bool up_before = LtdcLayer<2>::enabled();
    Ltdc::reload(LtdcReload::immediate);
    const bool up = LtdcLayer<2>::enabled();
    LtdcLayer<2>::disable();
    Ltdc::reload(LtdcReload::immediate);
    bench.verdict("a layer's enable is shadowed too: it reads clear until the reload takes it, "
                  "and that is what lets a program install a whole new picture - both layers, "
                  "their windows and their buffers - in one frame boundary",
                  !up_before && up && !LtdcLayer<2>::enabled());

    print(serial, "  what a layer's WINDOW and BLENDING really paint is not readable by any "
          "register of this block: the position counter and the underrun flag are what a "
          "program can see, and letter n is what a person looks at", crlf);

    (void)display_ready();
}

// =============================================================================
// n - something to look at
// =============================================================================

void tn_picture() {
    (void)display_ready();
    Dma2d::init();

    // Eight vertical bars of the primaries, painted by the accelerator
    // one rectangle at a time - which is also the shape a display
    // library's "fill this box" verb has.
    constexpr uint32_t bars[8] = {
        argb8888(255, 0, 0, 0),     argb8888(255, 0, 0, 255),   argb8888(255, 0, 255, 0),
        argb8888(255, 0, 255, 255), argb8888(255, 255, 0, 0),   argb8888(255, 255, 0, 255),
        argb8888(255, 255, 255, 0), argb8888(255, 255, 255, 255)};
    const uint16_t bar_width = panel_width / 8u;
    bool painted = true;
    for (uint32_t i = 0; i < 8u; ++i) {
        const Dma2dOutput out{
            .address = sdram_base + fb16_offset + 2u * (i * bar_width),
            .line_offset = static_cast<uint16_t>(panel_width - bar_width),
            .format = Dma2dOutputColor::rgb565};
        painted = painted &&
                  Dma2d::fill(out, bars[i],
                              Dma2dArea{.pixels = bar_width,
                                        .lines = static_cast<uint16_t>(panel_height - 40u)}) &&
                  Dma2d::wait();
    }
    print(serial, "  eight colour bars painted by the accelerator: ", painted ? "done" : "FAILED",
          crlf);
    bench.verdict("the accelerator paints a rectangle inside a wider surface, which is what its "
                  "line offset is for", painted);

    // A bar sliding along the bottom for three seconds, one step a
    // frame, with the frame counted on the console.
    const uint16_t strip_y = static_cast<uint16_t>(panel_height - 40u);
    const Dma2dOutput strip{.address = sdram_base + fb16_offset + 2u * strip_y * panel_width,
                            .line_offset = 0,
                            .format = Dma2dOutputColor::rgb565};
    line_events = 0;
    Ltdc::clear(ltdc_event_mask(LtdcEvent::line));
    Ltdc::interrupt(LtdcEvent::line, true);
    Ltdc::enable_interrupt();
    uint32_t steps = 0;
    const uint32_t start = millis();
    uint32_t previous_frame = line_events;
    while (millis() - start < 3000u) {
        if (line_events == previous_frame) {
            continue;
        }
        previous_frame = line_events;
        const uint16_t x = static_cast<uint16_t>((steps * 4u) % (panel_width - 32u));
        (void)Dma2d::fill(strip, argb8888(255, 0, 0, 0),
                          Dma2dArea{.pixels = panel_width, .lines = 40});
        (void)Dma2d::wait();
        const Dma2dOutput block{
            .address = sdram_base + fb16_offset + 2u * (strip_y * panel_width + x),
            .line_offset = static_cast<uint16_t>(panel_width - 32u),
            .format = Dma2dOutputColor::rgb565};
        (void)Dma2d::fill(block, argb8888(255, 255, 200, 0),
                          Dma2dArea{.pixels = 32, .lines = 32});
        (void)Dma2d::wait();
        ++steps;
    }
    const uint32_t frames = line_events;
    Ltdc::interrupt(LtdcEvent::line, false);
    Ltdc::disable_interrupt();
    const uint32_t underran = Ltdc::flag(LtdcEvent::fifo_underrun) ? 1u : 0u;
    print(serial, "  the bar stepped ", steps, " times over ", frames, " frames in three "
          "seconds; the FIFO underrun stood ", underran, " time(s)", crlf);
    bench.verdict("a rectangle repainted once a frame keeps up with the frame rate and starves "
                  "nothing", steps > 100u && steps + 4u >= frames && underran == 0u);
    print(serial, "  the panel now shows eight colour bars and a yellow block near the bottom - "
          "which is the one thing in this suite that a person and not a register confirms",
          crlf);
}

// =============================================================================
// the menu
// =============================================================================

void banner() {
    print(serial, crlf, "test_stm32f4_ltdc - the display tier: the LCD-TFT controller and the "
                        "Chrom-Art accelerator", crlf);
    bench.menu();
}

}   // namespace

extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// The controller's global vector: the line and register-reload events,
/// and NOT the error vector's two - the four flags share one register
/// and the body takes only the pair this vector carries.
extern "C" void LTDC_IRQHandler() {
    const uint32_t standing = brio::Ltdc::isr(brio::ltdc_global_events);
    if ((standing & brio::ltdc_event_mask(brio::LtdcEvent::line)) != 0u) {
        line_events = line_events + 1u;
        if (line_event_y == 0u) {
            line_event_y = brio::Ltdc::position().y;
        }
    }
    if ((standing & brio::ltdc_event_mask(brio::LtdcEvent::register_reload)) != 0u) {
        reload_events = reload_events + 1u;
    }
}

/// The controller's error vector: the FIFO underrun and the transfer
/// error. The same body with the other mask.
extern "C" void LTDC_ER_IRQHandler() {
    const uint32_t standing = brio::Ltdc::isr(brio::ltdc_error_events);
    if (standing != 0u) {
        error_events = error_events + 1u;
    }
}

extern "C" void DMA2D_IRQHandler() { (void)brio::Dma2d::isr(); }

int main() {
    // What the silicon held before a line of this program ran. The
    // gates are read out of the RCC, so reading them does not open
    // them; the blocks' own registers need their clock, and what they
    // hold behind it is their reset value.
    boot.ltdc_gate = brio::Ltdc::clock();
    boot.dma2d_gate = brio::Dma2d::clock();
    boot.pllsai_on = brio::Rcc::pllsai_ready();
    brio::Ltdc::clock(true);
    brio::Dma2d::clock(true);
    boot.dark = read_registers();
    boot.d2cr = brio::Dma2d::regs().CR;
    boot.d2isr = brio::Dma2d::regs().ISR;
    boot.d2amtcr = brio::Dma2d::regs().AMTCR;
    boot.d2opfccr = brio::Dma2d::regs().OPFCCR;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);

    // The memory first - a read of a bank that never ran its sequence
    // hangs the machine (ES0206 2.3.3) - then the pixel clock, then the
    // pads, then the controller, and the panel last: its own controller
    // wants a running RGB interface when it is told to take one.
    claim_sdram_pads();
    boot.sdram_up = Sdram::initialize(clock, sdram_geometry, sdram_timing, sdram_boot);
    boot.clock_up = brio::Ltdc::pixel_clock(pixel_pll);
    // The same registers again, with LCD_CLK running and still nothing
    // of this program written into them: what the block really holds
    // out of reset.
    boot.clocked = read_registers();
    claim_panel_pads();
    GyroCs::output(true);   // the other device on SPI5, left alone
    LcdCs::output(true);
    LcdDcx::output(true);
    (void)brio::Ltdc::timing(panel_timing);
    brio::Ltdc::background(0, 0, 0);

    // NOTHING ENABLES A LAYER UNTIL THE MEMORY IS UP: the controller
    // would fetch a bank that never ran its sequence, and that read
    // hangs the machine with no fault.
    if (boot.sdram_up) {
        fb16.fill(rgb565(0, 0, 0));
        (void)display_ready();
    }

    (void)Host::init(clock);
    Host::sck_speed(PinSpeed::very_high);
    boot.panel_bytes = panel_init();
    boot.panel_up = boot.panel_bytes != 0u;

    brio::Dma2d::init();
    brio::enable_interrupts();

    bench.letter('a', "the two blocks, and what they held at reset", ta_blocks);
    bench.letter('b', "the pixel clock and the frame rate", tb_pixel_clock);
    bench.letter('c', "what the drivers refuse", tc_refusals);
    bench.letter('d', "the panel in its RGB mode, and the one answer it gives", td_panel);
    bench.letter('e', "the timing registers", te_timing);
    bench.letter('f', "the position counter, and what a read of it costs", tf_position);
    bench.letter('g', "the line event", tg_line_event);
    bench.letter('h', "the shadow registers and the two reloads", th_shadow);
    bench.letter('i', "one layer out of the external memory", ti_one_layer);
    bench.letter('j', "the bandwidth: layers, depths and the accelerator", tj_bandwidth);
    bench.letter('k', "the accelerator's four modes, byte-exact", tk_modes);
    bench.letter('l', "the colour tables, both ways", tl_clut);
    bench.letter('m', "the layer geometry and the blending registers", tm_layer);
    bench.letter('n', "colour bars and a moving bar, for a human to look at", tn_picture);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED",
                    " sdram=", boot.sdram_up ? "up" : "FAILED",
                    " lcdclk=", boot.clock_up ? "locked" : "FAILED",
                    " panel=", boot.panel_bytes, " bytes", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
