// test_stm32f4_dsi - the reference bench suite for the STM32F4's DSI Host:
// the MIPI Display Serial Interface in front of the LTDC, its regulator,
// PLL and D-PHY, the generic interface a panel is spoken to through, the
// adapted command mode that turns an LTDC frame into memory writes, and
// video mode as the host streams it - stm32f4/dsi.hpp, with
// stm32f4/ltdc.hpp, stm32f4/dma2d.hpp and stm32f4/fmc.hpp under it.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENT IS A PANEL THAT ANSWERS, which is why this suite builds
// for the 32F469IDISCOVERY alone: its 4" 800x480 module (the MB1166) sits
// on the DSI host's two lanes, and a DSI panel can be READ - its
// controller reports its identity, its power mode, its pixel format, its
// own count of link errors, and THE CONTENT OF ITS FRAME MEMORY, over the
// same link that drives it. That is what makes this suite a measurement
// with no eyes: the verdicts are the panel's answers, the picture itself
// is read back pixel for pixel, and a human is asked to look only in the
// last letter.
//
//   the panel   the MB1166's controller, identified at boot by RDID1/2/3
//               (DAh, DBh, DCh) against the two the module was built
//               with - Orise's OTM8009A (ID1 40h, ID2 00h, ID3 00h) and
//               Novatek's NT35510 (00h, 80h, 00h); reset on PH7 (shared
//               with the touch controller), tearing effect on PJ2 (AF13 to
//               the wrapper, EXTI line 2 to the core), the backlight's boost
//               converter enabled by the panel's own CABC output, so
//               WRCTRLD's BL bit is the backlight switch (UM1932 4.14, R117
//               fitted). The NT35510's datasheet documents its DSI as a
//               COMMAND interface: the frame memory written by RAMWR/RAMWRC
//               (2Ch/3Ch) and read by RAMRD/RAMRDC (2Eh/3Eh), the standard
//               DCS set for everything else - and that is how the suite
//               drives it: the host's ADAPTED COMMAND MODE, one refresh per
//               frame, the tearing effect from the pin
//   the link    two lanes at 496 Mbit/s: the DSI PLL off the 8 MHz crystal
//               with IDF 1, NDIV 62, ODF 1 (the VCO at 992 MHz - 500 is not
//               exact with the PFD held inside both documents' ranges),
//               the lane byte clock at 62 MHz, the TX escape clock at
//               15.5 MHz, the D-PHY timings from DS11189 table 46; the
//               memory writes in high speed, every other command in low
//               power
//   the frame   800x480 at a 26.4 MHz pixel clock (PLLSAI N 132, R 5, the
//               LCD divider 2 over the main PLL's M of 4): HSA 2, HBP 20,
//               HFP 20 pixels, VSA 10, VBP 15, VFP 16 lines, a 16.6 ms
//               frame the wrapper lets the LTDC run once per refresh, cut
//               into DCS long writes of one line each (CMDSIZE 800); the
//               same numbers converted into lane byte clocks for the video
//               mode letter, non-burst with sync pulses, 24-bit pixels
//   the memory  the 128-Mbit SDRAM on FMC bank 1 at 0xC0000000, 32 bits
//               wide, brought up by stm32f4/fmc.hpp before a pixel is
//               fetched; a 16-bit and a 32-bit frame buffer in it, both
//               painted by the accelerator and never read by the CPU
//   the clocks  the core at 180 MHz, the memory at 90 MHz, PCLK2 at 90 MHz
//
// What is exercised, letter by letter:
//   a  the block: the gate closed at reset and the registers behind it,
//      the configuration read back, the vector, the lane byte clock's source
//   b  the regulator and the PLL: the PLL dropped and relocked with its lock
//      time, its unlock flag, the regulator dropped and back, the host's
//      registers through it, the panel still answering after
//   c  what the driver refuses, and that a refusal writes nothing
//   d  the D-PHY and the host at rest: the lanes in their stop state, the
//      FIFOs empty, no error standing, a read answered with nothing else
//      on the link
//   e  THE PANEL'S IDENTITY OVER THE LINK: the three ID bytes, the
//      controller named, the reads repeatable, the panel's own DSI error
//      count at zero
//   f  the panel's registers written and read back: pixel format, address
//      mode, tearing effect, brightness, the backlight control, display on
//      and off, sleep in and out - each through its read-back command
//   g  the errors and the interrupt: a forced error read and cleared by the
//      read, the vector entered once, the wrapper's flags, the end of a
//      refresh
//   h  VIDEO MODE, AS THE HOST STREAMS IT: the frame out of the memory and
//      the pattern generator, judged by the error registers - and the
//      panel's memory read back unchanged, because this panel's DSI takes
//      commands and not a stream; the read that runs out in video mode
//      with the LTDC stopped
//   i  THE ADAPTED COMMAND MODE: one LTDC frame as memory writes, the
//      refresh timed, the refresh rate over a second, no error on either
//      side - and THE FRAME READ BACK out of the panel's memory, pixel for
//      pixel at sixteen points and along a run, against what the
//      accelerator painted
//   j  the tearing effect line: the panel's pulses counted on EXTI line 2
//      and by the wrapper from the pin, and the automatic refresh paced by
//      them
//   k  the wrapper's shutdown and colour mode packets, against the panel's
//      power mode
//   l  the ultra-low-power state on the data lanes, entered and left
//   m  colour bars and a moving bar, for a human to look at
//   n  (by name only) A FINGER CHASES A SQUARE: the module's touch
//      controller read over I2C1 while a white square is refreshed into
//      the panel at ten random places and a human taps it - the raw touch
//      coordinates against the square's, the orientation of the touch
//      frame relative to the display frame solved from them (which of
//      the eight axis swaps and mirrors fits), a red cross drawn where
//      each tap was understood to be, the error in pixels
//
// build: boards = f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma2d.hpp"
#include "stm32f4/dsi.hpp"
#include "stm32f4/exti.hpp"
#include "stm32f4/fmc.hpp"
#include "stm32f4/i2c.hpp"
#include "stm32f4/ltdc.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "kernel/borrowed.hpp"
#include "util/i2c_bus.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the console --------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
using Serial = Uart<3, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- the external memory ----------------------------------------------------------

using Sdram = FmcSdram<1>;

/// The IS42S32400F in the FMC suite's words: 4096 rows x 256 columns x 4
/// internal banks x 32 bits, the -6 grade's seven delays at 90 MHz.
constexpr SdramConfig sdram_geometry{.columns = SdramColumns::eight,
                                     .rows = SdramRows::twelve,
                                     .width = SdramWidth::bits32,
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
constexpr uint32_t sdram_refresh = sdram_refresh_count(sdram_hz, 64'000u, 4096u);
constexpr SdramInit sdram_boot{.power_up_us = 100,
                               .refresh_cycles = 8,
                               .mode_register = sdram_mode_register(SdramCas::three),
                               .refresh_count = static_cast<uint16_t>(sdram_refresh),
                               .both_banks = false};

/// The memory's 54 pads at AF12, per port, as the MB1189's MCU sheet wires
/// them (the FMC suite's own list).
void claim_sdram_pads() {
    constexpr PinConfig cfg{.pull = PinPull::up, .open_drain = false, .speed = PinSpeed::very_high};
    Port<'C'>::configure_mask(1u << 0, PinMode::alternate, cfg, PinFunction::af12);
    Port<'D'>::configure_mask((1u << 0) | (1u << 1) | (1u << 8) | (1u << 9) | (1u << 10) | (1u << 14) | (1u << 15),
                              PinMode::alternate, cfg, PinFunction::af12);
    Port<'E'>::configure_mask((1u << 0) | (1u << 1) | 0xFF80u, PinMode::alternate, cfg, PinFunction::af12);
    Port<'F'>::configure_mask(0x003Fu | (1u << 11) | 0xF000u, PinMode::alternate, cfg, PinFunction::af12);
    Port<'G'>::configure_mask((1u << 0) | (1u << 1) | (1u << 4) | (1u << 5) | (1u << 8) | (1u << 15),
                              PinMode::alternate, cfg, PinFunction::af12);
    Port<'H'>::configure_mask((1u << 2) | (1u << 3) | 0xFF00u, PinMode::alternate, cfg, PinFunction::af12);
    Port<'I'>::configure_mask(0x00FFu | (1u << 9) | (1u << 10), PinMode::alternate, cfg, PinFunction::af12);
}

// ---- the frame: the LTDC's numbers and the link's ------------------------------------

constexpr uint16_t panel_width = 800;
constexpr uint16_t panel_height = 480;

/// The OTM8009A datasheet's typical porches (table 6.4.2.1: HS 2, HBP and
/// HFP 20 pixel clocks, VS 10, VBP 15, VFP 16 lines), which the NT35510's
/// minima (VBP 5, VFP 2, HBP 5, HFP 2) sit under. The polarities are the
/// LTDC's business only: on this board they never reach a pad, the
/// wrapper is told the same values, and its halt of the LTDC in the
/// adapted command mode lands on the VSYNC's assertion, the falling edge.
constexpr LtdcTiming panel_timing{.hsync = 2,
                                  .hbp = 20,
                                  .width = panel_width,
                                  .hfp = 20,
                                  .vsync = 10,
                                  .vbp = 15,
                                  .height = panel_height,
                                  .vfp = 16,
                                  .hsync_polarity = LtdcPolarity::active_low,
                                  .vsync_polarity = LtdcPolarity::active_low,
                                  .de_polarity = LtdcPolarity::active_low,
                                  .clock_edge = LtdcClockEdge::direct};

constexpr uint32_t pixel_hz = 26'400'000u;
constexpr LcdClockConfig pixel_pll = lcd_clock_config_for(SysClock::root_hz, pixel_hz, SysClock::pll.m);
static_assert(pixel_pll.pll.n == 132 && pixel_pll.pll.r == 5 && pixel_pll.divider == LcdClockDivider::div2);
constexpr uint32_t frame_pixels = ltdc_frame_pixels(panel_timing);
constexpr uint32_t frame_rate_mhz = ltdc_frame_rate_mhz(pixel_hz, panel_timing);   // milli-hertz
constexpr uint32_t frame_period_us = static_cast<uint32_t>((static_cast<uint64_t>(frame_pixels) * 1'000'000u) / pixel_hz);
static_assert(frame_rate_mhz > 59'000u && frame_rate_mhz < 61'000u);

constexpr uint32_t bit_rate_hz = 496'000'000u;
/// The link's configuration: the memory writes of a refresh are DCS long
/// writes and go in high speed; every other command stays in low power.
constexpr DsiConfig dsi_cfg = [] {
    DsiConfig c = dsi_config_for(SysClock::root_hz, bit_rate_hz);
    c.host.long_writes_low_power = false;
    c.phy.clock_lane_hs = false;   // low power until the panel has been reset and spoken to
    return c;
}();
static_assert(dsi_cfg.pll.ndiv == 62 && dsi_cfg.pll.idf == 1 && dsi_cfg.pll.odf == 1);
constexpr uint32_t lane_byte_hz = dsi_lane_byte_hz(bit_rate_hz);
constexpr DsiVideoTiming link_timing = dsi_video_timing(panel_timing, pixel_hz, lane_byte_hz);
static_assert(dsi_line_fits(link_timing, DsiColor::rgb888, DsiLanes::two));
constexpr DsiVideoConfig video_cfg{};   // non-burst, sync pulses, every return to LP allowed

// ---- the pads that are the panel's -------------------------------------------------

using PanelReset = Pin<'H', 7>;   // shared with the touch controller (UM1932 4.14)
using PanelTe = Pin<'J', 2>;      // DSIHOST_TE on AF13 (DS11189 table 12), EXTI line 2
using TeInt = ExtInt<PanelTe>;

// ---- the module's touch controller, for the letter that wants a finger -----------------

/// I2C1 on PB8/PB9 under the board's 1.5 k pull-ups, the FocalTech
/// controller at 0x38 (the I2C suite's peer): its touch data block,
/// TD_STATUS then the first point's XH/XL/YH/YL, five bytes from 0x02.
constexpr I2cPins touch_pins{.scl = {'B', 8, PinFunction::af4}, .sda = {'B', 9, PinFunction::af4}};
using Touch = I2cHost<1, touch_pins>;
constexpr uint8_t touch_addr = 0x38;
constexpr uint8_t touch_td_status = 0x02;
volatile bool touch_done = false;

// ---- the frame buffers --------------------------------------------------------------

constexpr uint32_t sdram_base = Sdram::base;
constexpr uint32_t fb16_offset = 0x0000'0000u;   // 768 KB
constexpr uint32_t fb32_offset = 0x0010'0000u;   // 1.5 MB
const LtdcFramebuffer<uint16_t> fb16{Sdram::at<uint16_t>(fb16_offset), panel_width, panel_height, 0};

// ---- the panel's command set: DCS, as both datasheets spell it ------------------------

constexpr uint8_t dcs_rdnumed = 0x05;    // the panel's count of DSI errors
constexpr uint8_t dcs_rddpm = 0x0A;      // power mode: bit 4 SLPOUT, bit 2 DISON
constexpr uint8_t dcs_rddmadctr = 0x0B;
constexpr uint8_t dcs_rddcolmod = 0x0C;
constexpr uint8_t dcs_rddsm = 0x0E;      // signal mode: bit 7 TEON
constexpr uint8_t dcs_slpin = 0x10;
constexpr uint8_t dcs_slpout = 0x11;
constexpr uint8_t dcs_dispoff = 0x28;
constexpr uint8_t dcs_dispon = 0x29;
constexpr uint8_t dcs_caset = 0x2A;
constexpr uint8_t dcs_paset = 0x2B;      // the NT35510 spells it RASET
constexpr uint8_t dcs_ramrd = 0x2E;      // the frame memory read back
constexpr uint8_t dcs_teoff = 0x34;
constexpr uint8_t dcs_teon = 0x35;
constexpr uint8_t dcs_madctr = 0x36;
constexpr uint8_t dcs_colmod = 0x3A;
constexpr uint8_t dcs_wrdisbv = 0x51;
constexpr uint8_t dcs_rddisbv = 0x52;
constexpr uint8_t dcs_wrctrld = 0x53;
constexpr uint8_t dcs_rdctrld = 0x54;
constexpr uint8_t dcs_rdid1 = 0xDA;
constexpr uint8_t dcs_rdid2 = 0xDB;
constexpr uint8_t dcs_rdid3 = 0xDC;

constexpr uint8_t colmod_24bit = 0x77;   // VIPF 7, IFPF 7: 24 bits a pixel on both interfaces
constexpr uint8_t colmod_16bit = 0x55;
constexpr uint8_t madctr_landscape = 0x60;   // MV and MX: rows and columns exchanged, mirrored back
constexpr uint8_t wrctrld_backlight = 0x24;  // BCTRL and BL: the brightness block on, the backlight on
constexpr uint8_t rddpm_slpout = 0x10;
constexpr uint8_t rddpm_dison = 0x04;
constexpr uint8_t rddsm_teon = 0x80;

/// A module's controller by the three bytes it answers RDID1..3 with.
struct PanelFacts {
    const char* name;
    uint8_t id1, id2, id3;
};
constexpr PanelFacts known_panels[] = {
    {"OTM8009A (Orise)", 0x40, 0x00, 0x00},
    {"NT35510 (Novatek)", 0x00, 0x80, 0x00},
};
constexpr uint8_t unknown_panel = 0xFF;

// ---- the rulers -----------------------------------------------------------------------

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// Microseconds of kernel time from the tick and SysTick's own counter,
/// read coherently.
uint32_t micros() {
    uint32_t t1, t2, v;
    do {
        t1 = Ticker::ticks();
        v = SysTick->VAL;
        t2 = Ticker::ticks();
    } while (t1 != t2);
    const uint32_t period = systick_period();
    return t1 * 1000u + ((period - 1u - v) * 1000u) / period;
}

uint32_t millis() { return P::now(); }

void wait_ms(uint32_t ms) {
    const uint32_t start = millis();
    while (millis() - start < ms) {
    }
}

void wait_us(uint32_t us) {
    const uint32_t t0 = micros();
    while (micros() - t0 < us) {
    }
}

const char* status_name(DsiStatus s) {
    switch (s) {
        case DsiStatus::ok: return "ok";
        case DsiStatus::refused: return "refused";
        case DsiStatus::timeout: return "timeout";
        default: return "error";
    }
}

// ---- what the registers held before this program touched them --------------------------

struct BootState {
    bool gate = false;
    uint32_t vr = 0, cr = 0, ccr = 0, pconfr = 0, mcr = 0, psr = 0, gpsr = 0;
    uint32_t wrpcr = 0, wisr = 0, wcfgr = 0, vmcr = 0, cltcr = 0, dltcr = 0;
    bool byte_clock_pllr = false;
    bool sdram_up = false;
    bool pixel_clock_up = false;
    bool dsi_up = false;
    bool colour_ok = false, mode_ok = false;
    uint32_t regulator_us = 0, lock_us = 0;
    uint32_t e0_link = 0, e0_reset = 0, e0_ids = 0, e0_init = 0, e0_frame = 0;
    uint8_t panel = unknown_panel;
    uint8_t id[3] = {0, 0, 0};
    DsiStatus id_status = DsiStatus::refused;
    uint32_t first_refresh_us = 0;
};
BootState boot;

// ---- the events ------------------------------------------------------------------------

volatile uint32_t line_events = 0;
volatile uint32_t dsi_irq_entries = 0;
volatile uint32_t dsi_irq_errors0 = 0;
volatile uint32_t dsi_irq_errors1 = 0;
volatile uint32_t dsi_irq_wrapper = 0;
volatile uint32_t te_edges = 0;
/// What the first read after each re-enable of the host found in ISR0,
/// ORed over the suite: the panel's acknowledge for the lanes' move.
uint32_t settled_errors = 0;
uint32_t settles = 0;

// =============================================================================
// The panel
// =============================================================================

/// RESX low for 20 ms, then the time both datasheets want before the
/// first command: the OTM8009A's 2 ms for its OTP reload is under the
/// NT35510's 120 ms after RESX is released, so 150 ms serves both.
void panel_reset() {
    PanelReset::output();
    PanelReset::clear();
    wait_ms(20);
    PanelReset::set();
    wait_ms(150);
}

/// One register of the panel, read back through the link: the byte, or
/// 0xFF with `st` saying why.
uint8_t panel_read(uint8_t command, DsiStatus& st) {
    uint8_t b = 0xFF;
    st = Dsi::dcs_read(command, &b, 1);
    return b;
}
uint8_t panel_read(uint8_t command) {
    DsiStatus st = DsiStatus::ok;
    return panel_read(command, st);
}

/// The three identity bytes, and the controller they name.
uint8_t identify(uint8_t (&id)[3], DsiStatus& st) {
    st = Dsi::dcs_read(dcs_rdid1, &id[0], 1);
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_read(dcs_rdid2, &id[1], 1);
    }
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_read(dcs_rdid3, &id[2], 1);
    }
    if (st != DsiStatus::ok) {
        return unknown_panel;
    }
    for (uint8_t i = 0; i < sizeof known_panels / sizeof known_panels[0]; ++i) {
        if (known_panels[i].id1 == id[0] && known_panels[i].id2 == id[1] && known_panels[i].id3 == id[2]) {
            return i;
        }
    }
    return unknown_panel;
}

const char* panel_name(uint8_t index) {
    return index == unknown_panel ? "an unknown controller" : known_panels[index].name;
}

/// The write window: the columns and the rows the next memory write or
/// read covers (CASET, RASET/PASET), four bytes each, big-endian.
DsiStatus panel_window(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1) {
    const uint8_t columns[4] = {static_cast<uint8_t>(x0 >> 8), static_cast<uint8_t>(x0 & 0xFFu),
                                static_cast<uint8_t>(x1 >> 8), static_cast<uint8_t>(x1 & 0xFFu)};
    const uint8_t rows[4] = {static_cast<uint8_t>(y0 >> 8), static_cast<uint8_t>(y0 & 0xFFu),
                             static_cast<uint8_t>(y1 >> 8), static_cast<uint8_t>(y1 & 0xFFu)};
    DsiStatus st = Dsi::dcs_write(dcs_caset, columns, 4);
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_write(dcs_paset, rows, 4);
    }
    return st;
}

DsiStatus panel_whole_window() { return panel_window(0, panel_width - 1u, 0, panel_height - 1u); }

/// The standard DCS bring-up, the same for both controllers: out of
/// sleep, the pixel format, the landscape address mode over the whole
/// 800x480 window, the display on, the backlight through the brightness
/// block. The memory writes themselves are the adapted command mode's.
DsiStatus panel_init() {
    DsiStatus st = Dsi::dcs_write(dcs_slpout);
    wait_ms(120);
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_write(dcs_colmod, colmod_24bit);
    }
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_write(dcs_madctr, madctr_landscape);
    }
    if (st == DsiStatus::ok) {
        st = panel_whole_window();
    }
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_write(dcs_dispon);
    }
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_write(dcs_wrctrld, wrctrld_backlight);
    }
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_write(dcs_wrdisbv, 0xFF);
    }
    wait_ms(20);
    return st;
}

/// `n` pixels of the panel's frame memory from (x, y) along a row, as the
/// panel returns them (three bytes a pixel in the 24-bit format, and one
/// more byte read in case the answer leads with one), the whole window
/// put back afterwards.
DsiStatus panel_read_pixels(uint16_t x, uint16_t y, uint16_t n, uint8_t* bytes) {
    DsiStatus st = panel_window(x, static_cast<uint16_t>(x + n - 1u), y, y);
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_read(dcs_ramrd, bytes, static_cast<uint16_t>(3u * n + 1u));
    }
    (void)panel_whole_window();
    return st;
}

// =============================================================================
// The display tier, brought up in 18.14.1's order
// =============================================================================

LtdcLayerConfig one_layer(LtdcPixelFormat format, uint32_t framebuffer) {
    LtdcLayerConfig c{};
    c.window = LtdcWindow{0, 0, panel_width, panel_height};
    c.format = format;
    c.framebuffer = framebuffer;
    c.constant_alpha = 255;
    c.blend_source = LtdcBlend1::constant_alpha;
    c.blend_below = LtdcBlend2::one_minus_constant_alpha;
    return c;
}

constexpr uint16_t blanking_line = static_cast<uint16_t>(panel_timing.vsync + panel_timing.vbp + panel_timing.height);

/// The LTDC showing a surface, its line event armed on the first blanking
/// line, enabled - and, in the adapted command mode, held by the wrapper
/// until a refresh lets it run one frame.
bool ltdc_show(LtdcPixelFormat format, uint32_t framebuffer) {
    LtdcLayer<2>::disable();
    const bool ok = LtdcLayer<1>::configure(one_layer(format, framebuffer), panel_timing);
    LtdcLayer<1>::enable();
    (void)Ltdc::line_interrupt(blanking_line);
    Ltdc::reload(LtdcReload::immediate);
    Ltdc::interrupt(LtdcEvent::line, true);
    Ltdc::enable_interrupt();
    Ltdc::enable();
    return ok;
}

bool ltdc_show16() { return ltdc_show(LtdcPixelFormat::rgb565, sdram_base + fb16_offset); }
bool ltdc_show32() { return ltdc_show(LtdcPixelFormat::argb8888, sdram_base + fb32_offset); }

/// The LTDC stopped at the frame boundary, so that no pixel packet is cut
/// short: the line event is armed on the first blanking line, and the
/// disable right after it lands inside the vertical front porch.
void ltdc_stop() {
    if (Ltdc::enabled() && !Dsi::in_command_mode()) {
        const uint32_t target = line_events + 1u;
        const uint32_t deadline = millis() + 60u;
        while (line_events < target && static_cast<int32_t>(millis() - deadline) < 0) {
        }
    }
    Ltdc::disable();
    Ltdc::interrupt(LtdcEvent::line, false);
    Ltdc::disable_interrupt();
}

/// One refresh of the adapted command mode: the wrapper lets the LTDC run
/// a frame, the host turns it into memory writes, ERIF closes it. The
/// time it took, or 0 when the end of refresh never came.
uint32_t refresh(uint32_t timeout_ms = 100) {
    Dsi::clear_wrapper_flags(DSI_WISR_ERIF);
    const uint32_t t0 = micros();
    const uint32_t deadline = millis() + timeout_ms;
    Dsi::ltdc_flow(true);
    while ((Dsi::wrapper_flags() & DSI_WISR_ERIF) == 0u) {
        if (static_cast<int32_t>(millis() - deadline) > 0) {
            return 0u;
        }
    }
    const uint32_t took = micros() - t0;
    return took == 0u ? 1u : took;
}

/// After the host's enable: the first read, which carries the
/// acknowledge the panel sends for the lanes' transition (AE6, a false
/// control error, at the first bus turnaround after a reset on either
/// side - measured), and the error registers read clear.
void link_settle() {
    wait_ms(20);
    uint8_t b = 0;
    (void)Dsi::dcs_read(dcs_rdid1, &b, 1);
    settled_errors |= Dsi::errors0();
    settles = settles + 1u;
    (void)Dsi::errors1();
}

/// Eight vertical bars into a surface, by the accelerator: white, yellow,
/// cyan, green, magenta, red, blue, black - the pattern generator's own
/// order (RM0386 18.11.2). The bar a column falls in is what a pixel read
/// back out of the panel is judged against: the CPU never reads the
/// surface (ES0206 2.3.5).
constexpr uint32_t bar_colours[8] = {argb8888(255, 255, 255, 255), argb8888(255, 255, 255, 0),
                                     argb8888(255, 0, 255, 255),   argb8888(255, 0, 255, 0),
                                     argb8888(255, 255, 0, 255),   argb8888(255, 255, 0, 0),
                                     argb8888(255, 0, 0, 255),     argb8888(255, 0, 0, 0)};
constexpr uint16_t bar_width = panel_width / 8u;

constexpr uint32_t bar_colour_at(uint16_t x) { return bar_colours[x / bar_width] & 0x00FFFFFFu; }

bool paint_bars(uint32_t base, Dma2dOutputColor format, uint8_t bytes_per_pixel) {
    bool painted = true;
    for (uint32_t i = 0; i < 8u; ++i) {
        const Dma2dOutput out{.address = base + bytes_per_pixel * (i * bar_width),
                              .line_offset = static_cast<uint16_t>(panel_width - bar_width),
                              .format = format};
        painted = painted && Dma2d::fill(out, bar_colours[i], Dma2dArea{.pixels = bar_width, .lines = panel_height}) &&
                  Dma2d::wait();
    }
    return painted;
}
bool paint_bars16() { return paint_bars(sdram_base + fb16_offset, Dma2dOutputColor::rgb565, 2); }
bool paint_bars32() { return paint_bars(sdram_base + fb32_offset, Dma2dOutputColor::argb8888, 4); }

/// The link brought up: the regulator, the PLL, the D-PHY, the host, the
/// LTDC interface in 24 bits, the adapted command mode with a line per
/// packet, the tearing effect from the pin, the LTDC halted on the
/// VSYNC's falling edge, the host enabled. Times the two waits.
bool link_up() {
    Dsi::clock(true);
    Dsi::disable();
    uint32_t t0 = micros();
    if (!Dsi::regulator(true)) {
        return false;
    }
    boot.regulator_us = micros() - t0;
    t0 = micros();
    if (!Dsi::pll(SysClock::root_hz, dsi_cfg.pll)) {
        return false;
    }
    boot.lock_us = micros() - t0;
    if (!Dsi::phy(dsi_cfg.phy, dsi_cfg.bit_rate_hz) || !Dsi::host(dsi_cfg.host)) {
        return false;
    }
    boot.colour_ok = Dsi::colour(DsiColor::rgb888, panel_timing);
    boot.mode_ok = Dsi::command_mode(panel_width);
    (void)Dsi::te_source(true, false);
    (void)Dsi::adapted_refresh(false, false);
    Dsi::enable();
    return boot.colour_ok && boot.mode_ok;
}

/// Into video mode and back: the host disabled around the mode change,
/// the registers of the other mode untouched.
void enter_video_mode() {
    Dsi::disable();
    (void)Dsi::video(video_cfg, link_timing);
    Dsi::enable();
}
void enter_command_mode() {
    Dsi::disable();
    (void)Dsi::command_mode(panel_width);
    Dsi::enable();
}

// =============================================================================
// a - the block
// =============================================================================

void ta_block() {
    print(serial, "  this part: DSI host ", dsi_present() ? "present" : "absent", ", LTDC ",
          ltdc_present() ? "present" : "absent", ", vector ", static_cast<int>(Dsi::irq), crlf);
    bench.verdict("the reserve says this part has the DSI host, in front of an LTDC",
                  dsi_present() && ltdc_present());
    bench.verdict("the vector is 92, the last of this class's table (RM0386 table 43)",
                  static_cast<int>(Dsi::irq) == 92);

    print(serial, "  at reset: the APB2 gate ", boot.gate ? "OPEN" : "closed", "; VR ", hex(boot.vr), " CR ",
          hex(boot.cr), " CCR ", hex(boot.ccr), " PCONFR ", hex(boot.pconfr), " MCR ", hex(boot.mcr), " PSR ",
          hex(boot.psr), " GPSR ", hex(boot.gpsr), crlf, "            WRPCR ", hex(boot.wrpcr), " WISR ",
          hex(boot.wisr), " WCFGR ", hex(boot.wcfgr), " VMCR ", hex(boot.vmcr), " CLTCR ", hex(boot.cltcr),
          " DLTCR ", hex(boot.dltcr), crlf);
    bench.verdict("the gate is closed at reset", !boot.gate);
    bench.verdict("the host wakes disabled, in command mode, on two lanes (CR 0, MCR 1, PCONFR 1)",
                  boot.cr == 0u && boot.mcr == 1u && boot.pconfr == 1u);
    bench.verdict("the D-PHY's defined status bits wake clear with the regulator off (PSR reads 0x1400: two "
                  "reserved bits set, none of 18.15.40's 0x1528) and the FIFOs as 18.15.26's 0x15",
                  (boot.psr & 0x1FEu) == 0u && boot.gpsr == 0x15u);
    bench.verdict("the regulator, the PLL and every wrapper flag wake clear",
                  boot.wrpcr == 0u && boot.wisr == 0u && boot.wcfgr == 0u);

    const DsiPllConfig pll = Dsi::pll_config();
    const DsiPhyTiming t = Dsi::phy_timing();
    print(serial, "  after init: PLL NDIV ", pll.ndiv, " IDF ", pll.idf, " ODF ", pll.odf, " (", bit_rate_hz / 1'000'000u,
          " Mbit/s a lane, the lane byte clock ", lane_byte_hz / 1'000'000u, " MHz), UIX4 ", Dsi::uix4(), ", ",
          Dsi::lanes() == DsiLanes::two ? "two lanes" : "one lane", ", escape divider ", Dsi::escape_divider(),
          ", short commands in ", Dsi::commands_low_power() ? "low power" : "high speed", ", long writes in ",
          Dsi::long_writes_low_power() ? "low power" : "high speed", ", channel ", Dsi::virtual_channel(), crlf);
    print(serial, "  lane timings: clock HS2LP ", t.clock_hs2lp, " LP2HS ", t.clock_lp2hs, " data HS2LP ",
          t.data_hs2lp, " LP2HS ", t.data_lp2hs, " stop wait ", t.stop_wait, " max read ", t.max_read_time,
          " (lane byte clocks)", crlf);
    bench.verdict("the PLL triple reads back as solved: NDIV 62, IDF 1, ODF 1",
                  pll.ndiv == 62u && pll.idf == 1u && pll.odf == 1u);
    bench.verdict("UIX4 holds 8: 2.016 ns is 8.06 quarters, rounded down (18.14.2)", Dsi::uix4() == 8u);
    bench.verdict("the clock lane's two times are EQUAL, the larger of the pair (ES0321 2.8.2)",
                  t.clock_hs2lp == t.clock_lp2hs && t.clock_hs2lp == 35u);
    bench.verdict("the data lane's times and the escape divider are the arithmetic's",
                  t.data_lp2hs == 21u && t.data_hs2lp == 17u && Dsi::escape_divider() == 4u);
    bench.verdict("the short commands go in low power and the long writes in high speed, as configured",
                  Dsi::commands_low_power() && !Dsi::long_writes_low_power());
    print(serial, "  the mode: ", Dsi::in_command_mode() ? "command" : "video", ", WCFGR ", hex(Dsi::regs().WCFGR),
          " (DSIM ", (Dsi::regs().WCFGR & DSI_WCFGR_DSIM) != 0u ? "1" : "0", ", TESRC ",
          (Dsi::regs().WCFGR & DSI_WCFGR_TESRC) != 0u ? "pin" : "link", ", AR ",
          (Dsi::regs().WCFGR & DSI_WCFGR_AR) != 0u ? "1" : "0", "), LCCR ", Dsi::regs().LCCR, " pixels a command",
          crlf);
    bench.verdict("the adapted command mode reads back: MCR.CMDM and WCFGR.DSIM set, the tearing effect from the "
                  "pin, no automatic refresh, 800 pixels a memory write",
                  Dsi::in_command_mode() && (Dsi::regs().WCFGR & (DSI_WCFGR_DSIM | DSI_WCFGR_TESRC | DSI_WCFGR_AR)) ==
                                                (DSI_WCFGR_DSIM | DSI_WCFGR_TESRC) &&
                      Dsi::regs().LCCR == panel_width);
    bench.verdict("the lane byte clock comes from the D-PHY (DCKCFGR.DSISEL clear), as at reset",
                  !boot.byte_clock_pllr && !Dsi::byte_clock_from_pllr());
    bench.verdict("the colour coding reads back 24-bit in the host and the wrapper alike",
                  Dsi::colour() == DsiColor::rgb888 &&
                      ((Dsi::regs().WCFGR & DSI_WCFGR_COLMUX_Msk) >> DSI_WCFGR_COLMUX_Pos) == 5u);
}

// =============================================================================
// b - the regulator and the PLL
// =============================================================================

void tb_regulator_pll() {
    print(serial, "  at boot the regulator was ready in ", boot.regulator_us, " us and the PLL locked in ",
          boot.lock_us, " us (tLOCK 200 us at most, DS11189 table 47)", crlf);
    bench.verdict("the regulator came ready and the PLL locked at boot", boot.dsi_up);
    bench.verdict("the lock took under 200 us", boot.lock_us <= 200u);

    // The host quiet, the PLL dropped: PLLLS falls. PLLUIF does NOT rise
    // for a stop the program asked for - measured, the flag is a lost
    // lock's.
    const uint32_t pconfr = Dsi::regs().PCONFR;
    const uint32_t cmcr = Dsi::regs().CMCR;
    const uint32_t pcr = Dsi::regs().PCR;
    const uint32_t lcolcr = Dsi::regs().LCOLCR;
    const uint32_t lccr = Dsi::regs().LCCR;
    const uint32_t cltcr = Dsi::regs().CLTCR;
    Dsi::disable();
    Dsi::clear_wrapper_flags(DSI_WISR_PLLUIF | DSI_WISR_PLLLIF);
    Dsi::pll_off();
    wait_us(50);
    const bool unlocked = !Dsi::pll_locked();
    const uint32_t flags_off = Dsi::wrapper_flags();
    print(serial, "  PLL off: locked ", Dsi::pll_locked() ? "yes" : "no", ", WISR ", hex(flags_off),
          (flags_off & DSI_WISR_PLLUIF) != 0u ? " (the unlock flag rose)" : " (no unlock flag: a software stop is not a lost lock)",
          crlf);
    bench.verdict("with PLLEN cleared the lock status falls", unlocked);

    // A triple outside the ranges is refused, the PLL left as it is.
    const bool refused = !Dsi::pll(SysClock::root_hz, DsiPllConfig{.ndiv = 125, .idf = 2, .odf = 1});
    bench.verdict("a triple whose PFD input is under 8 MHz is refused and PLLEN stays clear",
                  refused && !Dsi::pll_on());

    Dsi::clear_wrapper_flags(DSI_WISR_PLLLIF);
    uint32_t t0 = micros();
    const bool relocked = Dsi::pll(SysClock::root_hz, dsi_cfg.pll);
    const uint32_t relock_us = micros() - t0;
    const uint32_t flags_on = Dsi::wrapper_flags();
    print(serial, "  PLL back: locked in ", relock_us, " us, WISR ", hex(flags_on), crlf);
    bench.verdict("the PLL relocks, in under 200 us, and the lock flag rises",
                  relocked && relock_us <= 200u && (flags_on & DSI_WISR_PLLLIF) != 0u);

    // The regulator too: off, the D-PHY with it, and back.
    Dsi::pll_off();
    (void)Dsi::regulator(false);
    wait_us(100);
    const bool reg_down = !Dsi::regulator_ready();
    Dsi::clear_wrapper_flags(DSI_WISR_RRIF);
    t0 = micros();
    const bool reg_up = Dsi::regulator(true);
    const uint32_t reg_us = micros() - t0;
    const uint32_t reg_flags = Dsi::wrapper_flags();
    print(serial, "  regulator off: ready ", reg_down ? "no" : "STILL", "; on again in ", reg_us, " us, WISR ",
          hex(reg_flags), crlf);
    bench.verdict("REGEN cleared takes the ready status down; set again it comes back with its flag",
                  reg_down && reg_up && (reg_flags & DSI_WISR_RRIF) != 0u);

    // The link whole again: the host's registers where they were, and the
    // panel still there.
    const bool lock_again = Dsi::pll(SysClock::root_hz, dsi_cfg.pll);
    Dsi::enable();
    const bool kept = Dsi::regs().PCONFR == pconfr && Dsi::regs().CMCR == cmcr && Dsi::regs().PCR == pcr &&
                      Dsi::regs().LCOLCR == lcolcr && Dsi::regs().LCCR == lccr && Dsi::regs().CLTCR == cltcr;
    bench.verdict("the host's configuration survives its disable (CR.EN = 0 holds it under reset, the "
                  "registers keep their values)", kept);
    link_settle();
    uint8_t id[3] = {0, 0, 0};
    DsiStatus st = DsiStatus::ok;
    const uint8_t which = identify(id, st);
    print(serial, "  the link back up: the panel answers ", hex(id[0]), " ", hex(id[1]), " ", hex(id[2]), " (",
          status_name(st), "), ", panel_name(which), crlf);
    bench.verdict("with the PLL relocked and the host re-enabled the panel answers its identity again",
                  lock_again && st == DsiStatus::ok && which == boot.panel);
    const uint32_t took = refresh();
    bench.verdict("... and takes a frame", took != 0u);
}

// =============================================================================
// c - what the driver refuses
// =============================================================================

void tc_refusals() {
    // Every configuring verb refuses while the host is enabled, and
    // writes nothing.
    const uint32_t pconfr = Dsi::regs().PCONFR;
    const uint32_t ccr = Dsi::regs().CCR;
    const uint32_t vlcr = Dsi::regs().VLCR;
    const uint32_t lcolcr = Dsi::regs().LCOLCR;
    const uint32_t lccr = Dsi::regs().LCCR;
    const bool phy_r = !Dsi::phy(dsi_cfg.phy, dsi_cfg.bit_rate_hz);
    const bool host_r = !Dsi::host(dsi_cfg.host);
    const bool video_r = !Dsi::video(video_cfg, link_timing);
    const bool colour_r = !Dsi::colour(DsiColor::rgb565_1, panel_timing);
    const bool cmd_r = !Dsi::command_mode(16);
    const bool te_r = !Dsi::te_source(false);
    const bool ar_r = !Dsi::adapted_refresh(true, true);
    bench.verdict("phy, host, video, colour, command_mode, te_source and adapted_refresh all refuse while the host "
                  "is enabled",
                  Dsi::enabled() && phy_r && host_r && video_r && colour_r && cmd_r && te_r && ar_r);
    bench.verdict("... and none of them wrote a register",
                  Dsi::regs().PCONFR == pconfr && Dsi::regs().CCR == ccr && Dsi::regs().VLCR == vlcr &&
                      Dsi::regs().LCOLCR == lcolcr && Dsi::regs().LCCR == lccr);

    // With the host disabled the same verbs take, and the frame's rules
    // still refuse: a line shorter than its sync and porch, a line the
    // pixels do not fit, a field past its width.
    Dsi::disable();
    DsiVideoTiming bad = link_timing;
    bad.hline = static_cast<uint16_t>(bad.hsa + bad.hbp);
    const bool short_line = !Dsi::video(video_cfg, bad);
    bad = link_timing;
    bad.hline = 1200;   // 1203 of payload a lane plus 52 of sync and porch do not fit
    const bool tight_line = !Dsi::video(video_cfg, bad);
    bad = link_timing;
    bad.vsa = 1024;
    const bool wide_field = !Dsi::video(video_cfg, bad);
    bench.verdict("a line that holds no pixels, one the pixels do not fit, or a field past its bits is refused",
                  short_line && tight_line && wide_field);
    bench.verdict("... and VLCR still holds the frame", Dsi::regs().VLCR == vlcr);
    DsiVideoConfig vc = video_cfg;
    vc.chunks = 8192;
    bench.verdict("a chunk count past NUMC's thirteen bits is refused", !Dsi::video(vc, link_timing));
    const bool good = Dsi::video(video_cfg, link_timing);
    bench.verdict("the frame itself takes again with the host disabled", good);
    const bool back = Dsi::command_mode(panel_width);
    bench.verdict("... and so does the command mode", back && Dsi::in_command_mode());

    // The generic interface refuses while disabled.
    uint8_t b = 0;
    const DsiStatus w = Dsi::dcs_write(dcs_rddpm);
    const DsiStatus r = Dsi::dcs_read(dcs_rddpm, &b, 1);
    bench.verdict("a write and a read are refused while the host is disabled",
                  w == DsiStatus::refused && r == DsiStatus::refused);
    const bool host_r2 = !Dsi::host(DsiHostConfig{.escape_divider = 1});
    bench.verdict("an escape divider that stops the clock is refused", host_r2);
    Dsi::enable();
    const DsiStatus zero = Dsi::dcs_read(dcs_rddpm, &b, 0);
    bench.verdict("a read of zero bytes is refused", zero == DsiStatus::refused);
    link_settle();
}

// =============================================================================
// d - the D-PHY and the host at rest
// =============================================================================

void td_phy() {
    wait_ms(30);
    const DsiLaneStatus s = Dsi::lane_status();
    print(serial, "  lanes with no refresh in flight: clock stop ", s.clock_stop ? "1" : "0", " ulps ",
          s.clock_ulps ? "1" : "0", "; data0 stop ", s.data0_stop ? "1" : "0", " ulps ", s.data0_ulps ? "1" : "0",
          " rx-ulps ", s.data0_rx_ulps ? "1" : "0", "; data1 stop ", s.data1_stop ? "1" : "0", " ulps ",
          s.data1_ulps ? "1" : "0", "; direction ", s.direction_rx ? "rx" : "tx", crlf);
    bench.verdict("both data lanes sit in their stop state, in ULPS neither, the bus ours",
                  s.data0_stop && s.data1_stop && !s.data0_ulps && !s.data1_ulps && !s.direction_rx);
    bench.verdict("the clock lane is NOT in stop state: DPCC keeps it in high speed", !s.clock_stop);
    print(serial, "  FIFOs: command ", Dsi::command_fifo_empty() ? "empty" : "NOT EMPTY", ", write ",
          Dsi::write_fifo_empty() ? "empty" : "NOT EMPTY", ", read ", Dsi::read_fifo_empty() ? "empty" : "NOT EMPTY",
          ", read busy ", Dsi::read_busy() ? "YES" : "no", ", refresh busy ", Dsi::busy() ? "YES" : "no", crlf);
    bench.verdict("the three FIFOs are empty, no read and no refresh in flight",
                  Dsi::command_fifo_empty() && Dsi::write_fifo_empty() && Dsi::read_fifo_empty() && !Dsi::read_busy() &&
                      !Dsi::busy());
    const uint32_t e0 = Dsi::errors0();
    const uint32_t e1 = Dsi::errors1();
    print(serial, "  ISR0 along the bring-up: after the link ", hex(boot.e0_link), ", the panel's reset ", hex(boot.e0_reset),
          ", the identity ", hex(boot.e0_ids), ", the init ", hex(boot.e0_init), ", the first frame ", hex(boot.e0_frame),
          "; now ISR0 ", hex(e0), " ISR1 ", hex(e1), "; the first read after each of the ", settles,
          " re-enables so far found ", hex(settled_errors), crlf);
    bench.verdict("no acknowledge error along the bring-up: the panel reset and spoken to with no stream on the "
                  "lanes reports nothing at its first turnaround",
                  boot.e0_link == 0u && boot.e0_reset == 0u && boot.e0_ids == 0u && boot.e0_init == 0u &&
                      boot.e0_frame == 0u);
    bench.verdict("no error stands now, after the letters so far", e0 == 0u && e1 == 0u);

    // In command mode a read is answered with nothing else on the link:
    // no stream, no refresh.
    uint8_t id = 0xEE;
    const DsiStatus st = Dsi::dcs_read(dcs_rdid1, &id, 1);
    print(serial, "  command mode, the LTDC held: RDID1 ", hex(id), " (", status_name(st), ")", crlf);
    bench.verdict("in command mode a read is answered with no stream and no refresh in flight",
                  st == DsiStatus::ok && id == boot.id[0]);
}

// =============================================================================
// e - the panel's identity over the link
// =============================================================================

void te_identity() {
    uint8_t id[3] = {0, 0, 0};
    DsiStatus st = DsiStatus::ok;
    const uint8_t which = identify(id, st);
    print(serial, "  RDID1 ", hex(id[0]), " RDID2 ", hex(id[1]), " RDID3 ", hex(id[2]), " (", status_name(st), "): ",
          panel_name(which), crlf);
    bench.verdict("the three identity reads complete over the link", st == DsiStatus::ok);
    bench.verdict("the triple names one of the two controllers the MB1166 was built with", which != unknown_panel);
    bench.verdict("... the same one boot found", which == boot.panel);

    // Sixteen times running: a link that is right is right every time.
    bool stable = true;
    for (uint32_t i = 0; i < 16u && stable; ++i) {
        uint8_t again[3] = {0, 0, 0};
        DsiStatus s2 = DsiStatus::ok;
        (void)identify(again, s2);
        stable = s2 == DsiStatus::ok && again[0] == id[0] && again[1] == id[1] && again[2] == id[2];
    }
    bench.verdict("sixteen identity reads running answer the same three bytes", stable);

    // What the panel says about itself and about the link.
    DsiStatus s_pm = DsiStatus::ok, s_err = DsiStatus::ok, s_sm = DsiStatus::ok;
    const uint8_t pm = panel_read(dcs_rddpm, s_pm);
    const uint8_t sm = panel_read(dcs_rddsm, s_sm);
    const uint8_t errors = panel_read(dcs_rdnumed, s_err);
    print(serial, "  power mode ", hex(pm), " (sleep out ", (pm & rddpm_slpout) != 0u ? "yes" : "no", ", display on ",
          (pm & rddpm_dison) != 0u ? "yes" : "no", "), signal mode ", hex(sm), ", DSI errors counted by the panel ",
          errors, crlf);
    bench.verdict("the panel reports itself out of sleep with the display on, as the bring-up left it",
                  s_pm == DsiStatus::ok && (pm & (rddpm_slpout | rddpm_dison)) == (rddpm_slpout | rddpm_dison));
    bench.verdict("the panel's own count of DSI errors is zero", s_err == DsiStatus::ok && errors == 0u);
    bench.verdict("the host's error registers agree", Dsi::errors0() == 0u && Dsi::errors1() == 0u);
}

// =============================================================================
// f - the panel's registers written and read back
// =============================================================================

void tf_registers() {
    // The pixel format: 16 bits, then 24 again; RDDCOLMOD carries IFPF in
    // its low three bits.
    (void)Dsi::dcs_write(dcs_colmod, colmod_16bit);
    const uint8_t cm16 = panel_read(dcs_rddcolmod);
    (void)Dsi::dcs_write(dcs_colmod, colmod_24bit);
    const uint8_t cm24 = panel_read(dcs_rddcolmod);
    print(serial, "  COLMOD 55h -> RDDCOLMOD ", hex(cm16), "; COLMOD 77h -> ", hex(cm24), crlf);
    bench.verdict("the pixel format written is the one read back: 5 for 16 bits, 7 for 24",
                  (cm16 & 0x07u) == 0x05u && (cm24 & 0x07u) == 0x07u);

    // The address mode.
    (void)Dsi::dcs_write(dcs_madctr, 0x00);
    const uint8_t ma0 = panel_read(dcs_rddmadctr);
    (void)Dsi::dcs_write(dcs_madctr, madctr_landscape);
    const uint8_t ma1 = panel_read(dcs_rddmadctr);
    print(serial, "  MADCTR 00h -> RDDMADCTR ", hex(ma0), "; MADCTR 60h -> ", hex(ma1), crlf);
    bench.verdict("the address mode's MY/MX/MV bits read back as written", (ma0 & 0xE0u) == 0x00u && (ma1 & 0xE0u) == 0x60u);

    // The tearing effect line.
    (void)Dsi::dcs_write(dcs_teon, 0x00);
    const uint8_t sm_on = panel_read(dcs_rddsm);
    (void)Dsi::dcs_write(dcs_teoff);
    const uint8_t sm_off = panel_read(dcs_rddsm);
    print(serial, "  TEON -> RDDSM ", hex(sm_on), "; TEOFF -> ", hex(sm_off), crlf);
    bench.verdict("TEON and TEOFF show in the signal mode's bit 7", (sm_on & rddsm_teon) != 0u && (sm_off & rddsm_teon) == 0u);

    // The brightness and the control block. RDDISBV answers the PREVIOUS
    // value straight after a write and the new one a frame later.
    (void)Dsi::dcs_write(dcs_wrdisbv, 0x80);
    const uint8_t bv80_now = panel_read(dcs_rddisbv);
    wait_ms(40);
    const uint8_t bv80 = panel_read(dcs_rddisbv);
    (void)Dsi::dcs_write(dcs_wrdisbv, 0xFF);
    wait_ms(40);
    const uint8_t bvff = panel_read(dcs_rddisbv);
    (void)Dsi::dcs_write(dcs_wrctrld, 0x00);
    const uint8_t cd0 = panel_read(dcs_rdctrld);
    (void)Dsi::dcs_write(dcs_wrctrld, wrctrld_backlight);
    const uint8_t cd1 = panel_read(dcs_rdctrld);
    print(serial, "  WRDISBV 80h -> RDDISBV ", hex(bv80_now), " at once, ", hex(bv80), " a frame later; FFh -> ", hex(bvff),
          " a frame later; WRCTRLD 00h -> RDCTRLD ", hex(cd0), "; 24h -> ", hex(cd1), crlf);
    bench.verdict("the brightness value reads back as written a frame after the write", bv80 == 0x80u && bvff == 0xFFu);
    bench.verdict("the control display byte's BCTRL and BL bits read back as written",
                  (cd0 & 0x24u) == 0x00u && (cd1 & 0x24u) == 0x24u);

    // The display off and on, sleep in and out: RDDPM's two bits.
    (void)Dsi::dcs_write(dcs_dispoff);
    wait_ms(40);
    const uint8_t pm_off = panel_read(dcs_rddpm);
    (void)Dsi::dcs_write(dcs_dispon);
    wait_ms(40);
    const uint8_t pm_on = panel_read(dcs_rddpm);
    print(serial, "  DISPOFF -> RDDPM ", hex(pm_off), "; DISPON -> ", hex(pm_on), crlf);
    bench.verdict("DISPOFF and DISPON show in the power mode's DISON bit",
                  (pm_off & rddpm_dison) == 0u && (pm_on & rddpm_dison) != 0u);
    (void)Dsi::dcs_write(dcs_slpin);
    wait_ms(120);
    const uint8_t pm_sleep = panel_read(dcs_rddpm);
    (void)Dsi::dcs_write(dcs_slpout);
    wait_ms(120);
    const uint8_t pm_awake = panel_read(dcs_rddpm);
    print(serial, "  SLPIN -> RDDPM ", hex(pm_sleep), "; SLPOUT -> ", hex(pm_awake), crlf);
    bench.verdict("SLPIN and SLPOUT show in the power mode's SLPOUT bit, 120 ms apart",
                  (pm_sleep & rddpm_slpout) == 0u && (pm_awake & rddpm_slpout) != 0u);
    // A sleep may take the display with it: the bring-up's tail again,
    // and a frame.
    (void)Dsi::dcs_write(dcs_dispon);
    (void)refresh();
    const uint8_t errors = panel_read(dcs_rdnumed);
    bench.verdict("after all of the above the panel's DSI error count is still zero", errors == 0u);
    bench.verdict("... and the host's error registers too", Dsi::errors0() == 0u && Dsi::errors1() == 0u);
}

// =============================================================================
// g - the errors and the interrupt
// =============================================================================

void tg_errors() {
    // A forced acknowledge error stands in ISR0, and the read that shows it
    // clears it (18.13.2). The store into FIR takes a moment to show.
    Dsi::force_errors(DSI_FIR0_FAE0, 0u);
    wait_us(20);
    const uint32_t first = Dsi::errors0();
    const uint32_t second = Dsi::errors0();
    print(serial, "  FIR0.FAE0: ISR0 reads ", hex(first), " then ", hex(second), crlf);
    bench.verdict("a forced error appears in ISR0 and the read clears it", (first & DSI_ISR0_AE0) != 0u && second == 0u);
    Dsi::force_errors(0u, DSI_FIR1_FTOHSTX);
    wait_us(20);
    const uint32_t f1 = Dsi::errors1();
    const uint32_t f1b = Dsi::errors1();
    bench.verdict("the same in ISR1", (f1 & DSI_ISR1_TOHSTX) != 0u && f1b == 0u);

    // The vector: one error enabled, forced, the body entered once.
    dsi_irq_entries = 0;
    dsi_irq_errors0 = 0;
    Dsi::error_interrupts(DSI_IER0_AE1IE, 0u);
    Dsi::enable_interrupt();
    Dsi::force_errors(DSI_FIR0_FAE1, 0u);
    wait_us(100);
    const uint32_t entries = dsi_irq_entries;
    const uint32_t seen = dsi_irq_errors0;
    Dsi::force_errors(DSI_FIR0_FAE2, 0u);   // not enabled: no entry
    wait_us(100);
    const uint32_t entries_masked = dsi_irq_entries;
    const uint32_t leftover = Dsi::errors0();
    Dsi::error_interrupts(0u, 0u);
    Dsi::disable_interrupt();
    print(serial, "  AE1 enabled and forced: ", entries, " entr", entries == 1u ? "y" : "ies", ", the body saw ISR0 ",
          hex(seen), "; AE2 forced unmasked: ", entries_masked - entries, " more; ISR0 then ", hex(leftover), crlf);
    bench.verdict("the vector is entered once for an enabled error and the body reports it",
                  entries == 1u && (seen & DSI_ISR0_AE1) != 0u);
    bench.verdict("an error whose enable is clear raises no interrupt and stands until read",
                  entries_masked == entries && (leftover & DSI_ISR0_AE2) != 0u);

    // The wrapper's flags: cleared through WIFCR, and reported by the body
    // with the PLL's lock interrupt enabled. A software stop of the PLL
    // raises no unlock flag (letter b), so only the lock is expected.
    Dsi::disable();
    Dsi::clear_wrapper_flags(DSI_WISR_PLLLIF | DSI_WISR_PLLUIF);
    const uint32_t cleared = Dsi::wrapper_flags() & (DSI_WISR_PLLLIF | DSI_WISR_PLLUIF);
    dsi_irq_entries = 0;
    dsi_irq_wrapper = 0;
    Dsi::wrapper_interrupts(DSI_WISR_PLLLIF | DSI_WISR_PLLUIF);
    Dsi::enable_interrupt();
    Dsi::pll_off();
    wait_us(100);
    const uint32_t after_off = dsi_irq_wrapper;
    (void)Dsi::pll(SysClock::root_hz, dsi_cfg.pll);
    wait_us(100);
    const uint32_t after_on = dsi_irq_wrapper;
    Dsi::wrapper_interrupts(0u);
    Dsi::disable_interrupt();
    Dsi::enable();
    link_settle();
    print(serial, "  wrapper flags cleared to ", hex(cleared), "; PLL off: the body saw ", hex(after_off), "; PLL on: ",
          hex(after_on), " (", dsi_irq_entries, " entries)", crlf);
    bench.verdict("WIFCR clears the lock and unlock flags", cleared == 0u);
    bench.verdict("the lock enters the vector and the body reports it",
                  (after_on & DSI_WISR_PLLLIF) != 0u && dsi_irq_entries >= 1u);

    // The end-of-refresh flag through the vector too.
    dsi_irq_entries = 0;
    dsi_irq_wrapper = 0;
    Dsi::clear_wrapper_flags(DSI_WISR_ERIF);
    Dsi::wrapper_interrupts(DSI_WISR_ERIF);
    Dsi::enable_interrupt();
    Dsi::ltdc_flow(true);
    wait_ms(40);
    Dsi::wrapper_interrupts(0u);
    Dsi::disable_interrupt();
    print(serial, "  a refresh with ERIE: ", dsi_irq_entries, " entr", dsi_irq_entries == 1u ? "y" : "ies", ", the body saw ",
          hex(dsi_irq_wrapper), crlf);
    bench.verdict("the end of a refresh enters the vector once and the body reports it",
                  dsi_irq_entries == 1u && (dsi_irq_wrapper & DSI_WISR_ERIF) != 0u);
}

// =============================================================================
// h - video mode, as the host streams it
// =============================================================================

/// One pixel of the panel's memory, decoded from what RAMRD answers; the
/// alignment (a leading byte or none) is `offset`.
uint32_t pixel_at(const uint8_t* bytes, uint16_t i, uint8_t offset) {
    const uint8_t* p = bytes + offset + 3u * i;
    return (static_cast<uint32_t>(p[0]) << 16) | (static_cast<uint32_t>(p[1]) << 8) | p[2];
}

void th_video_mode() {
    // A pixel of the panel's memory before: the 16-bit bars refreshed.
    (void)paint_bars16();
    (void)ltdc_show16();
    (void)refresh();
    uint8_t before[4] = {0, 0, 0, 0};
    const DsiStatus s_before = panel_read_pixels(0, 0, 1, before);

    // The 16-bit bars streamed in video mode for a second: no error on
    // the host, the panel's own count still zero - and its memory where
    // it was, because this panel's DSI takes commands, not a stream.
    enter_video_mode();
    (void)Dsi::errors1();
    Ltdc::clear(ltdc_error_events);
    line_events = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 1000u) {
    }
    const uint32_t frames = line_events;
    const uint32_t e1 = Dsi::errors1();
    const bool underrun = Ltdc::flag(LtdcEvent::fifo_underrun);
    uint8_t id = 0;
    const DsiStatus st = Dsi::dcs_read(dcs_rdid1, &id, 1);
    DsiStatus s_err = DsiStatus::ok;
    const uint8_t errors = panel_read(dcs_rdnumed, s_err);
    print(serial, "  video mode, one second: ", frames, " frames (", frame_rate_mhz / 1000u, ".", (frame_rate_mhz / 100u) % 10u,
          " expected), ISR1 ", hex(e1), ", LTDC underrun ", underrun ? "YES" : "no", "; RDID1 ", hex(id), " (", status_name(st),
          "), the panel's DSI error count ", errors, " (", status_name(s_err), ")", crlf);
    bench.verdict("the host streams the LTDC's frame at the pixel clock's rate: 60 frames a second, within one",
                  frames + 1u >= frame_rate_mhz / 1000u && frames <= frame_rate_mhz / 1000u + 1u);
    bench.verdict("no LTDC payload write error and no FIFO underrun over a second of frames",
                  (e1 & DSI_ISR1_LPWRE) == 0u && !underrun);
    bench.verdict("a DCS read is answered while the video streams, in a blanking period",
                  st == DsiStatus::ok && id == boot.id[0]);
    bench.verdict("the panel counted no link error over the video", s_err == DsiStatus::ok && errors == 0u);

    // The read that runs out: the LTDC stopped in video mode.
    ltdc_stop();
    wait_ms(30);
    uint8_t id2 = 0xEE;
    const DsiStatus stopped = Dsi::dcs_read(dcs_rdid1, &id2, 1);
    const uint32_t gpsr_after = Dsi::regs().GPSR;
    (void)ltdc_show16();
    wait_ms(40);
    const uint32_t gpsr_running = Dsi::regs().GPSR;
    id2 = 0xEE;
    const DsiStatus running = Dsi::dcs_read(dcs_rdid1, &id2, 1);
    print(serial, "  video mode, LTDC stopped: RDID1 ", status_name(stopped), ", GPSR ", hex(gpsr_after),
          "; LTDC started: GPSR ", hex(gpsr_running), ", RDID1 ", hex(id2), " (", status_name(running), ")", crlf);
    bench.verdict("in video mode with the LTDC stopped a read runs out unanswered (RCB standing)",
                  stopped == DsiStatus::timeout && (gpsr_after & DSI_GPSR_RCB) != 0u);
    bench.verdict("with the stream back the same read is answered", running == DsiStatus::ok && id2 == boot.id[0]);

    // The pattern generator: the host's own bars with the LTDC stopped.
    ltdc_stop();
    wait_ms(30);
    (void)Dsi::errors1();
    Dsi::pattern(DsiPattern::vertical_bars);
    wait_ms(500);
    const uint32_t e1v = Dsi::errors1();
    Dsi::pattern(DsiPattern::horizontal_bars);
    wait_ms(500);
    const uint32_t e1h = Dsi::errors1();
    Dsi::pattern(DsiPattern::off);
    print(serial, "  the pattern generator, vertical then horizontal bars, half a second each: ISR1 ", hex(e1v), " then ",
          hex(e1h), crlf);
    bench.verdict("the host streams its own colour bars with the LTDC stopped and raises no error",
                  !Dsi::pattern_on() && e1v == 0u && e1h == 0u);

    // Back in command mode: the panel's memory as it was before a second
    // of video and the patterns.
    enter_command_mode();
    link_settle();
    (void)ltdc_show16();
    uint8_t after[4] = {0, 0, 0, 0};
    const DsiStatus s_after = panel_read_pixels(0, 0, 1, after);
    const uint8_t errors_after = panel_read(dcs_rdnumed);
    print(serial, "  the panel's pixel (0, 0): ", hex(before[0]), " ", hex(before[1]), " ", hex(before[2]), " ", hex(before[3]),
          " before, ", hex(after[0]), " ", hex(after[1]), " ", hex(after[2]), " ", hex(after[3]), " after (",
          status_name(s_before), ", ", status_name(s_after), "); the panel's error count ", errors_after, crlf);
    bench.verdict("THIS PANEL'S DSI IS A COMMAND INTERFACE: a second of video-mode frames and the patterns leave "
                  "its memory untouched, and it counts no error for them",
                  s_before == DsiStatus::ok && s_after == DsiStatus::ok && before[0] == after[0] && before[1] == after[1] &&
                      before[2] == after[2] && before[3] == after[3] && errors_after == 0u);
    bench.verdict("the host's error registers are clear after it", Dsi::errors0() == 0u && Dsi::errors1() == 0u);
}

// =============================================================================
// i - the adapted command mode, and the frame read back
// =============================================================================

/// The sixteen sample points: the four corners, the middle, and a point
/// inside each of the eight bars near the top and near the bottom, and
/// the two sides of the first bar boundary.
struct Sample {
    uint16_t x, y;
};
constexpr Sample samples[16] = {{0, 0},   {799, 0},  {0, 479},  {799, 479}, {400, 240}, {50, 10},   {150, 10},  {250, 10},
                                {350, 10}, {450, 470}, {550, 470}, {650, 470}, {750, 470}, {99, 240}, {100, 240}, {700, 300}};

void ti_adapted_command_mode() {
    // The 32-bit bars into the panel, one refresh, timed.
    (void)paint_bars32();
    (void)ltdc_show32();
    (void)Dsi::errors1();
    Ltdc::clear(ltdc_error_events);
    const uint32_t took = refresh();
    const bool busy_after = Dsi::busy();
    const bool flow_after = Dsi::ltdc_flow();
    print(serial, "  one refresh of 800x480 32-bit pixels: ", took, " us (the LTDC frame is ", frame_period_us,
          " us), then BUSY ", busy_after ? "1" : "0", ", LTDCEN ", flow_after ? "1" : "0", crlf);
    bench.verdict("a refresh ends with the end-of-refresh flag, LTDCEN cleared by the wrapper and BUSY down",
                  took != 0u && !busy_after && !flow_after);
    bench.verdict("... in about one LTDC frame: the memory writes keep up with the pixel clock",
                  took != 0u && took >= frame_period_us - 2000u && took <= frame_period_us + 4000u);

    // A second of refreshes back to back.
    uint32_t count = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 1000u) {
        if (refresh() != 0u) {
            ++count;
        }
    }
    const uint32_t e1 = Dsi::errors1();
    const bool underrun = Ltdc::flag(LtdcEvent::fifo_underrun);
    DsiStatus s_err = DsiStatus::ok;
    const uint8_t errors = panel_read(dcs_rdnumed, s_err);
    print(serial, "  a second of refreshes: ", count, " frames, ISR1 ", hex(e1), ", LTDC underrun ", underrun ? "YES" : "no",
          ", the panel's DSI error count ", errors, " (", status_name(s_err), ")", crlf);
    bench.verdict("refreshes back to back reach 50 frames a second and more", count >= 50u);
    bench.verdict("no payload write error, no FIFO underrun, no error counted by the panel over them",
                  e1 == 0u && !underrun && s_err == DsiStatus::ok && errors == 0u);

    // THE FRAME READ BACK. Each sample point read as one pixel of the
    // panel's memory and compared with the bar the accelerator painted
    // there: three bytes a pixel, and the alignment - whether the answer
    // leads with a byte - decided by the first point and held for the
    // rest.
    uint8_t bytes[4] = {0, 0, 0, 0};
    uint8_t offset = 0xFF;
    uint32_t matched = 0;
    for (uint32_t i = 0; i < 16u; ++i) {
        for (uint8_t& b : bytes) {
            b = 0xEE;
        }
        const DsiStatus st = panel_read_pixels(samples[i].x, samples[i].y, 1, bytes);
        const uint32_t want = bar_colour_at(samples[i].x);
        if (offset == 0xFFu && st == DsiStatus::ok) {
            offset = pixel_at(bytes, 0, 0) == want ? 0u : pixel_at(bytes, 0, 1) == want ? 1u : 0xFFu;
        }
        const bool ok = st == DsiStatus::ok && offset != 0xFFu && pixel_at(bytes, 0, offset) == want;
        if (ok) {
            ++matched;
        }
        if (i < 5u || !ok) {
            print(serial, "  (", samples[i].x, ", ", samples[i].y, "): the panel answers ", hex(bytes[0]), " ", hex(bytes[1]),
                  " ", hex(bytes[2]), " ", hex(bytes[3]), " (", status_name(st), "), painted ", hex(want), ok ? "" : " MISMATCH",
                  crlf);
        }
    }
    print(serial, "  ", matched, " of 16 sample points match, the answer ",
          offset == 0u ? "starting with the pixel" : offset == 1u ? "leading with one byte" : "in neither alignment", crlf);
    bench.verdict("THE PICTURE IS IN THE PANEL: sixteen pixels read back out of its memory are the bars the "
                  "accelerator painted, 24 bits each", matched == 16u);

    // A run of sixteen pixels across a bar boundary, in one packet.
    uint8_t run[16 * 3 + 1];
    for (uint8_t& b : run) {
        b = 0xEE;
    }
    const uint16_t x0 = static_cast<uint16_t>(bar_width - 8u);
    const DsiStatus s_run = panel_read_pixels(x0, 100, 16, run);
    uint32_t run_ok = 0;
    const uint8_t off = offset == 0xFFu ? 0u : offset;
    for (uint16_t i = 0; i < 16u; ++i) {
        if (pixel_at(run, i, off) == bar_colour_at(static_cast<uint16_t>(x0 + i))) {
            ++run_ok;
        }
    }
    print(serial, "  sixteen pixels from (", x0, ", 100) in one read (", status_name(s_run), "): ", run_ok, " match - the first ",
          hex(pixel_at(run, 0, off)), ", the ninth ", hex(pixel_at(run, 8, off)), crlf);
    bench.verdict("a run of sixteen pixels across the white/yellow boundary reads back in one packet, every pixel right",
                  s_run == DsiStatus::ok && run_ok == 16u);

    // The same through the 16-bit surface: the LTDC's expansion of 565 to
    // 888 as the panel stores it, printed and judged on the pure colours.
    (void)paint_bars16();
    (void)ltdc_show16();
    (void)refresh();
    uint8_t px[4] = {0, 0, 0, 0};
    const DsiStatus s16 = panel_read_pixels(150, 10, 1, px);   // the yellow bar
    const uint32_t yellow = pixel_at(px, 0, off);
    print(serial, "  the 16-bit surface's yellow (565 FFE0h) reaches the panel as ", hex(yellow), crlf);
    bench.verdict("the 16-bit surface's yellow reads back as full red and green and no blue",
                  s16 == DsiStatus::ok && (yellow & 0xF8F800u) == 0xF8F800u && (yellow & 0x0000FFu) == 0u);
    bench.verdict("the host's error registers are clear after the reads", Dsi::errors0() == 0u && Dsi::errors1() == 0u);
}

// =============================================================================
// j - the tearing effect line and the automatic refresh
// =============================================================================

void tj_tearing() {
    // The panel's TE output on PJ2: counted on EXTI line 2 for half a
    // second, and by the wrapper from the same pin.
    (void)Dsi::dcs_write(dcs_teon, 0x00);
    wait_ms(30);
    (void)TeInt::select();
    (void)TeInt::configure(ExtiSense::rising);
    (void)TeInt::clear();
    te_edges = 0;
    (void)TeInt::arm(true);
    Nvic::enable(TeInt::irq());
    Dsi::clear_wrapper_flags(DSI_WISR_TEIF);
    uint32_t te_flags = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 500u) {
        if ((Dsi::wrapper_flags() & DSI_WISR_TEIF) != 0u) {
            te_flags = te_flags + 1u;
            Dsi::clear_wrapper_flags(DSI_WISR_TEIF);
        }
    }
    const uint32_t edges = te_edges;
    print(serial, "  TEON, half a second: ", edges, " rising edges on PJ2 (EXTI line 2), ", te_flags,
          " tearing effect flags in the wrapper from the pin", crlf);
    bench.verdict("the panel pulses its TE line once a frame of its own scan: 25..35 edges in half a second",
                  edges >= 25u && edges <= 35u);
    bench.verdict("the wrapper sees the same pulses from the pin (WCFGR.TESRC), within two",
                  te_flags + 2u >= edges && te_flags <= edges + 2u);

    // The automatic refresh: every TE edge launches a frame, so the
    // refresh rate is the panel's scan rate.
    Dsi::disable();
    (void)Dsi::adapted_refresh(true, false);
    Dsi::enable();
    link_settle();
    Dsi::clear_wrapper_flags(DSI_WISR_ERIF);
    uint32_t refreshes = 0;
    te_edges = 0;
    t0 = millis();
    while (millis() - t0 < 1000u) {
        if ((Dsi::wrapper_flags() & DSI_WISR_ERIF) != 0u) {
            refreshes = refreshes + 1u;
            Dsi::clear_wrapper_flags(DSI_WISR_ERIF);
        }
    }
    const uint32_t edges_1s = te_edges;
    Dsi::disable();
    (void)Dsi::adapted_refresh(false, false);
    Dsi::enable();
    link_settle();
    const uint32_t e1 = Dsi::errors1();
    print(serial, "  automatic refresh on TE for a second: ", refreshes, " end-of-refresh flags against ", edges_1s,
          " TE edges, ISR1 ", hex(e1), crlf);
    bench.verdict("with WCFGR.AR the wrapper refreshes once per tearing effect pulse, within two of the edges counted",
                  refreshes + 2u >= edges_1s && refreshes <= edges_1s + 2u && refreshes >= 50u);
    bench.verdict("... with no payload write error", e1 == 0u);

    Nvic::disable(TeInt::irq());
    (void)TeInt::arm(false);
    (void)Dsi::dcs_write(dcs_teoff);
    wait_ms(30);
    te_edges = 0;
    (void)TeInt::clear();
    (void)TeInt::arm(true);
    Nvic::enable(TeInt::irq());
    wait_ms(200);
    Nvic::disable(TeInt::irq());
    (void)TeInt::arm(false);
    print(serial, "  TEOFF, 200 ms: ", te_edges, " edges", crlf);
    bench.verdict("TEOFF silences the line", te_edges == 0u);
}

// =============================================================================
// k - the wrapper's shutdown and colour mode packets
// =============================================================================

void tk_wrapper_packets() {
    const uint8_t before = panel_read(dcs_rddpm);
    Dsi::shutdown(true);
    wait_ms(60);
    const uint8_t down = panel_read(dcs_rddpm);
    Dsi::shutdown(false);
    wait_ms(60);
    const uint8_t up = panel_read(dcs_rddpm);
    print(serial, "  RDDPM ", hex(before), "; WCR.SHTDN set (shutdown peripheral): ", hex(down),
          "; cleared (turn on peripheral): ", hex(up), crlf);
    print(serial, "  ", (down & rddpm_dison) == 0u ? "the panel's display went off on the shutdown packet"
                                                    : "the panel's power mode did not move: this controller does not act on the shutdown packet",
          crlf);
    bench.verdict("the shutdown and turn-on packets go out, the panel answering throughout",
                  (before & rddpm_dison) != 0u && (up & rddpm_dison) != 0u && Dsi::errors0() == 0u && Dsi::errors1() == 0u);
    Dsi::eight_colours(true);
    wait_ms(60);
    const uint32_t e1 = Dsi::errors1();
    const uint8_t eight = panel_read(dcs_rddpm);
    Dsi::eight_colours(false);
    wait_ms(60);
    const uint8_t errors = panel_read(dcs_rdnumed);
    print(serial, "  WCR.COLM set (colour mode on): RDDPM ", hex(eight), ", ISR1 ", hex(e1), "; cleared; the panel's error count ",
          errors, crlf);
    bench.verdict("the colour mode packets go out without an error on either side", e1 == 0u && errors == 0u);
}

// =============================================================================
// l - the ultra-low-power state
// =============================================================================

void tl_ulps() {
    wait_ms(30);
    Dsi::ulps(false, true, true);
    wait_us(200);
    const DsiLaneStatus in = Dsi::lane_status();
    Dsi::ulps(false, true, false);
    wait_ms(2);
    Dsi::regs().PUCR = 0u;
    const DsiLaneStatus out = Dsi::lane_status();
    print(serial, "  ULPS requested on the data lanes: data0 ulps ", in.data0_ulps ? "1" : "0", " stop ", in.data0_stop ? "1" : "0",
          ", data1 ulps ", in.data1_ulps ? "1" : "0", "; exit: data0 ulps ", out.data0_ulps ? "1" : "0", " stop ",
          out.data0_stop ? "1" : "0", ", data1 ulps ", out.data1_ulps ? "1" : "0", " stop ", out.data1_stop ? "1" : "0", crlf);
    bench.verdict("PUCR.URDL takes both data lanes into ULPS (UAN0/UAN1 fall) and out of stop state",
                  in.data0_ulps && in.data1_ulps && !in.data0_stop);
    bench.verdict("PUCR.UEDL brings them back to stop state", !out.data0_ulps && !out.data1_ulps && out.data0_stop && out.data1_stop);
    uint8_t id = 0;
    const DsiStatus st = Dsi::dcs_read(dcs_rdid1, &id, 1);
    bench.verdict("the panel answers after the excursion", st == DsiStatus::ok && id == boot.id[0]);
    const uint32_t took = refresh();
    bench.verdict("... and takes a frame", took != 0u);
}

// =============================================================================
// m - a picture for a human
// =============================================================================

void tm_picture() {
    const bool painted = paint_bars16();
    (void)ltdc_show16();
    const bool shown = refresh() != 0u;
    bench.verdict("eight colour bars painted by the accelerator and refreshed into the panel", painted && shown);
    print(serial, "  eight bars, white to black, for two seconds; then a bar slides along the bottom", crlf);
    wait_ms(2000);
    const uint16_t strip_y = static_cast<uint16_t>(panel_height - 40u);
    const Dma2dOutput strip{.address = sdram_base + fb16_offset + 2u * strip_y * panel_width,
                            .line_offset = 0,
                            .format = Dma2dOutputColor::rgb565};
    uint32_t steps = 0;
    const uint32_t start = millis();
    while (millis() - start < 3000u) {
        const uint16_t x = static_cast<uint16_t>((steps * 4u) % (panel_width - 40u));
        (void)Dma2d::fill(strip, argb8888(255, 0, 0, 0), Dma2dArea{.pixels = panel_width, .lines = 40});
        (void)Dma2d::wait();
        const Dma2dOutput bar{.address = strip.address + 2u * x,
                              .line_offset = static_cast<uint16_t>(panel_width - 40u),
                              .format = Dma2dOutputColor::rgb565};
        (void)Dma2d::fill(bar, argb8888(255, 255, 255, 255), Dma2dArea{.pixels = 40, .lines = 40});
        (void)Dma2d::wait();
        if (refresh() != 0u) {
            ++steps;
        }
    }
    print(serial, "  ", steps, " steps in three seconds, one refresh each", crlf);
    bench.verdict("the bar moved at the refresh rate for three seconds: 120 steps or more", steps >= 120u);
    (void)paint_bars16();
    (void)refresh();
}

// =============================================================================
// n - a finger chases a square (by name only)
// =============================================================================

/// One write-then-read tenure against the touch controller, driven by
/// hand as the I2C suite drives its peer: the status the engine ended
/// with, or 0xFF when it never answered.
uint8_t touch_read(uint8_t reg, uint8_t* into, uint8_t n) {
    const uint8_t cmd[1] = {reg};
    Touch::Request r{};
    r.addr = touch_addr;
    r.tx = lend<Lease::reply>(cmd);
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(into);
    r.rx_len = n;
    r.speed = I2cSpeed::standard_100k;
    touch_done = false;
    if (Touch::start(r)) {
        return Touch::status();
    }
    for (uint32_t spins = 8'000'000u; spins != 0u; --spins) {
        if (touch_done) {
            return Touch::status();
        }
    }
    return 0xFF;
}

/// A rectangle of the 16-bit surface, by the accelerator.
void fill16(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t argb) {
    const Dma2dOutput out{.address = sdram_base + fb16_offset + 2u * (static_cast<uint32_t>(y) * panel_width + x),
                          .line_offset = static_cast<uint16_t>(panel_width - w),
                          .format = Dma2dOutputColor::rgb565};
    (void)Dma2d::fill(out, argb, Dma2dArea{.pixels = w, .lines = h});
    (void)Dma2d::wait();
}

/// The eight ways a 480x800 touch frame can sit on an 800x480 display:
/// the axes swapped or not, each mirrored or not. A raw point mapped by
/// candidate `m`.
struct Mapped {
    int32_t x, y;
};
Mapped map_touch(uint8_t m, uint16_t tx, uint16_t ty) {
    const bool swap = (m & 4u) != 0u;
    const bool flip_x = (m & 2u) != 0u;
    const bool flip_y = (m & 1u) != 0u;
    int32_t x = swap ? ty : tx;
    int32_t y = swap ? tx : ty;
    if (flip_x) {
        x = static_cast<int32_t>(panel_width) - 1 - x;
    }
    if (flip_y) {
        y = static_cast<int32_t>(panel_height) - 1 - y;
    }
    return Mapped{x, y};
}
const char* mapping_name(uint8_t m) {
    static const char* const names[8] = {"x = tx, y = ty",             "x = tx, y = 479 - ty",
                                         "x = 799 - tx, y = ty",       "x = 799 - tx, y = 479 - ty",
                                         "x = ty, y = tx",             "x = ty, y = 479 - tx",
                                         "x = 799 - ty, y = tx",       "x = 799 - ty, y = 479 - tx"};
    return names[m & 7u];
}

struct Tap {
    uint16_t tx, ty;   // the controller's own coordinates
    uint16_t sx, sy;   // the square's centre on the display
};

/// The candidate with the least squared error over the taps so far, and
/// that error.
uint8_t best_mapping(const Tap* taps, uint8_t n, uint32_t& error) {
    uint8_t best = 0;
    error = 0xFFFFFFFFu;
    for (uint8_t m = 0; m < 8u; ++m) {
        uint32_t e = 0;
        for (uint8_t i = 0; i < n; ++i) {
            const Mapped p = map_touch(m, taps[i].tx, taps[i].ty);
            const int32_t dx = p.x - taps[i].sx;
            const int32_t dy = p.y - taps[i].sy;
            e += static_cast<uint32_t>(dx * dx + dy * dy);
        }
        if (e < error) {
            error = e;
            best = m;
        }
    }
    return best;
}

uint32_t isqrt32(uint32_t v) {
    uint32_t r = 0;
    for (uint32_t bit = 1u << 15; bit != 0u; bit >>= 1) {
        const uint32_t t = r | bit;
        if (t * t <= v) {
            r = t;
        }
    }
    return r;
}

void tn_finger_square() {
    if (!Touch::init(clock, I2cHostConfig{})) {
        bench.verdict("I2C1 comes up for the touch controller", false);
        return;
    }
    uint8_t td[5] = {0, 0, 0, 0, 0};
    const uint8_t probe = touch_read(touch_td_status, td, 5);
    bench.verdict("the touch controller answers at 0x38 on I2C1", probe == i2c_ok);
    if (probe != i2c_ok) {
        return;
    }

    constexpr uint16_t side = 60;
    constexpr uint8_t targets = 10;
    Tap taps[targets];
    uint8_t n = 0;
    uint32_t seed = 0x2545F491u ^ millis();
    const auto rnd = [&seed](uint32_t m) {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        return seed % m;
    };
    uint16_t sx = static_cast<uint16_t>(side / 2u + rnd(panel_width - side));
    uint16_t sy = static_cast<uint16_t>(side / 2u + rnd(panel_height - side));
    (void)ltdc_show16();
    const auto draw = [&](bool cross, int32_t cx, int32_t cy) {
        fill16(0, 0, panel_width, panel_height, argb8888(255, 0, 0, 0));
        fill16(static_cast<uint16_t>(sx - side / 2u), static_cast<uint16_t>(sy - side / 2u), side, side,
               argb8888(255, 255, 255, 255));
        const int32_t w = panel_width;
        const int32_t h = panel_height;
        if (cross && cx >= 12 && cx < w - 12 && cy >= 12 && cy < h - 12) {
            fill16(static_cast<uint16_t>(cx - 12), static_cast<uint16_t>(cy - 1), 25, 3, argb8888(255, 255, 0, 0));
            fill16(static_cast<uint16_t>(cx - 1), static_cast<uint16_t>(cy - 12), 3, 25, argb8888(255, 255, 0, 0));
        }
        (void)refresh();
    };
    draw(false, 0, 0);
    print(serial, "  TAP THE WHITE SQUARE, ten times - up to 90 s; from the fourth tap a red cross marks "
                  "where the tap was understood to be",
          crlf);

    uint32_t errors = 0, samples = 0;
    uint8_t idle_run = 0;
    bool down = false;
    uint8_t best = 0;
    const uint32_t start = millis();
    while (n < targets && millis() - start < 90'000u) {
        const uint32_t t = millis();
        const uint8_t st = touch_read(touch_td_status, td, 5);
        if (st != i2c_ok) {
            ++errors;
        } else {
            ++samples;
            const uint8_t points = td[0] & 0x0Fu;
            if (points == 0u) {
                if (idle_run < 255u) {
                    ++idle_run;
                }
                down = false;
            } else if (!down && idle_run >= 3u) {
                // A tap: the first sample with a point after three without.
                down = true;
                idle_run = 0;
                const uint16_t tx = static_cast<uint16_t>(((td[1] & 0x0Fu) << 8) | td[2]);
                const uint16_t ty = static_cast<uint16_t>(((td[3] & 0x0Fu) << 8) | td[4]);
                taps[n] = Tap{tx, ty, sx, sy};
                ++n;
                uint32_t err = 0;
                Mapped p{0, 0};
                if (n >= 3u) {
                    best = best_mapping(taps, n, err);
                    p = map_touch(best, tx, ty);
                }
                if (n >= 3u) {
                    print(serial, "  tap ", n, ": raw (", tx, ", ", ty, ") for the square at (", sx, ", ", sy,
                          ") -> understood at (", p.x, ", ", p.y, ")", crlf);
                } else {
                    print(serial, "  tap ", n, ": raw (", tx, ", ", ty, ") for the square at (", sx, ", ", sy, ")", crlf);
                }
                if (n < targets) {
                    uint16_t nx, ny;
                    do {
                        nx = static_cast<uint16_t>(side / 2u + rnd(panel_width - side));
                        ny = static_cast<uint16_t>(side / 2u + rnd(panel_height - side));
                    } while (static_cast<uint16_t>(nx > sx ? nx - sx : sx - nx) < 150u &&
                             static_cast<uint16_t>(ny > sy ? ny - sy : sy - ny) < 150u);
                    sx = nx;
                    sy = ny;
                    draw(n >= 3u, p.x, p.y);
                }
            } else {
                idle_run = 0;
            }
        }
        while (millis() - t < 10u) {
        }
    }

    uint32_t err = 0;
    best = best_mapping(taps, n, err);
    uint32_t worst = 0;
    for (uint8_t i = 0; i < n; ++i) {
        const Mapped p = map_touch(best, taps[i].tx, taps[i].ty);
        const int32_t dx = p.x - taps[i].sx;
        const int32_t dy = p.y - taps[i].sy;
        const uint32_t d = isqrt32(static_cast<uint32_t>(dx * dx + dy * dy));
        worst = d > worst ? d : worst;
    }
    const uint32_t mean = n != 0u ? isqrt32(err / n) : 0u;
    print(serial, "  ", n, " taps in ", samples, " samples, ", errors, " bus errors; the touch frame on the display: ",
          mapping_name(best), " (mapping ", best, "), error ", mean, " px rms, ", worst, " px at worst", crlf);
    bench.verdict("ten taps arrived", n == targets);
    // A fingertip is some 60 px wide on this glass and the square 60: a
    // tap understood within 60 px of the centre is a tap on the square.
    bench.verdict("one orientation of the touch frame puts every tap on its square: within 60 px of the centre",
                  n == targets && worst <= 60u);
    bench.verdict("no bus error over the polling", errors == 0u);
    fill16(0, 0, panel_width, panel_height, argb8888(255, 0, 0, 0));
    (void)paint_bars16();
    (void)refresh();
}

// =============================================================================
// the menu
// =============================================================================

void banner() {
    print(serial, crlf, "test_stm32f4_dsi - the DSI host in front of the LTDC, against the board's panel", crlf);
    bench.menu();
}

}  // namespace

// ---- the vectors ---------------------------------------------------------------------------

extern "C" void USART3_IRQHandler() { (void)Serial::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void LTDC_IRQHandler() {
    const uint32_t standing = brio::Ltdc::isr(brio::ltdc_global_events);
    if ((standing & brio::ltdc_event_mask(brio::LtdcEvent::line)) != 0u) {
        line_events = line_events + 1u;
    }
}
extern "C" void LTDC_ER_IRQHandler() { (void)brio::Ltdc::isr(brio::ltdc_error_events); }
extern "C" void DMA2D_IRQHandler() { (void)brio::Dma2d::isr(); }

extern "C" void DSI_IRQHandler() {
    const brio::DsiEvents e = brio::Dsi::isr();
    dsi_irq_entries = dsi_irq_entries + 1u;
    dsi_irq_errors0 = dsi_irq_errors0 | e.errors0;
    dsi_irq_errors1 = dsi_irq_errors1 | e.errors1;
    uint32_t w = 0;
    if (e.pll_locked) {
        w |= DSI_WISR_PLLLIF;
    }
    if (e.pll_unlocked) {
        w |= DSI_WISR_PLLUIF;
    }
    if (e.tearing_effect) {
        w |= DSI_WISR_TEIF;
    }
    if (e.end_of_refresh) {
        w |= DSI_WISR_ERIF;
    }
    dsi_irq_wrapper = dsi_irq_wrapper | w;
}

extern "C" void I2C1_EV_IRQHandler() {
    if (Touch::isr()) {
        touch_done = true;
    }
}
extern "C" void I2C1_ER_IRQHandler() {
    if (Touch::error_isr()) {
        touch_done = true;
    }
}

extern "C" void EXTI2_IRQHandler() {
    const uint32_t fired = brio::Exti::isr(TeInt::mask);
    if (TeInt::served(fired)) {
        te_edges = te_edges + 1u;
    }
}

int main() {
    // What the silicon held before a line of this program ran: the gate
    // read out of the RCC, the block's registers behind it opened.
    boot.gate = brio::Dsi::clock();
    brio::Dsi::clock(true);
    boot.vr = brio::Dsi::regs().VR;
    boot.cr = brio::Dsi::regs().CR;
    boot.ccr = brio::Dsi::regs().CCR;
    boot.pconfr = brio::Dsi::regs().PCONFR;
    boot.mcr = brio::Dsi::regs().MCR;
    boot.psr = brio::Dsi::regs().PSR;
    boot.gpsr = brio::Dsi::regs().GPSR;
    boot.wrpcr = brio::Dsi::regs().WRPCR;
    boot.wisr = brio::Dsi::regs().WISR;
    boot.wcfgr = brio::Dsi::regs().WCFGR;
    boot.vmcr = brio::Dsi::regs().VMCR;
    boot.cltcr = brio::Dsi::regs().CLTCR;
    boot.dltcr = brio::Dsi::regs().DLTCR;
    boot.byte_clock_pllr = brio::Dsi::byte_clock_from_pllr();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // 18.14.1's order: the memory first (a fetch from a bank that never ran
    // its sequence hangs the machine), the LTDC's clock and timing, the
    // layer described and the controller enabled - held by the wrapper in
    // the adapted command mode until a refresh - then the link, then the
    // panel, then the first frame.
    claim_sdram_pads();
    boot.sdram_up = Sdram::initialize(clock, sdram_geometry, sdram_timing, sdram_boot);
    brio::Ltdc::clock(true);
    boot.pixel_clock_up = brio::Ltdc::pixel_clock(pixel_pll);
    (void)brio::Ltdc::timing(panel_timing);
    brio::Ltdc::background(0, 0, 0);
    brio::Dma2d::init();
    PanelTe::function(brio::PinFunction::af13);
    boot.dsi_up = link_up();
    if (boot.sdram_up) {
        fb16.fill(0);
        (void)ltdc_show16();
    }
    boot.e0_link = brio::Dsi::errors0();
    panel_reset();
    boot.e0_reset = brio::Dsi::errors0();
    boot.panel = identify(boot.id, boot.id_status);
    boot.e0_ids = brio::Dsi::errors0();
    const DsiStatus panel_st = panel_init();
    boot.e0_init = brio::Dsi::errors0();
    // THE CLOCK LANE INTO HIGH SPEED ONLY NOW: the panel's receiver locks
    // onto the clock lane's LP-to-HS entry, and a panel reset under a clock
    // lane already in high speed takes no high-speed packet afterwards -
    // measured: every frame refreshed into it was lost, with no error on
    // either side, until the host was disabled and enabled again.
    brio::Dsi::clock_lane(true);
    if (boot.sdram_up) {
        (void)paint_bars16();
        boot.first_refresh_us = refresh();
    }
    boot.e0_frame = brio::Dsi::errors0();

    bench.letter('a', "the block", ta_block);
    bench.letter('b', "the regulator and the PLL", tb_regulator_pll);
    bench.letter('c', "what the driver refuses", tc_refusals);
    bench.letter('d', "the D-PHY and the host at rest", td_phy);
    bench.letter('e', "the panel's identity over the link", te_identity);
    bench.letter('f', "the panel's registers written and read back", tf_registers);
    bench.letter('g', "the errors and the interrupt", tg_errors);
    bench.letter('h', "video mode, as the host streams it", th_video_mode);
    bench.letter('i', "the adapted command mode, and the frame read back", ti_adapted_command_mode);
    bench.letter('j', "the tearing effect line and the automatic refresh", tj_tearing);
    bench.letter('k', "the wrapper's shutdown and colour mode packets", tk_wrapper_packets);
    bench.letter('l', "the ultra-low-power state on the data lanes", tl_ulps);
    bench.letter('m', "colour bars and a moving bar, for a human to look at", tm_picture);
    bench.letter('n', "a finger chases a square: the touch frame on the display", tn_finger_square, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
                    " sdram=", boot.sdram_up ? "up" : "FAILED", " lcdclk=", boot.pixel_clock_up ? "locked" : "FAILED",
                    " dsi=", boot.dsi_up ? "up" : "FAILED", " panel=", panel_name(boot.panel), " (", brio::hex(boot.id[0]),
                    " ", brio::hex(boot.id[1]), " ", brio::hex(boot.id[2]), ", ids ", status_name(boot.id_status), ", init ",
                    status_name(panel_st), ") first refresh=", boot.first_refresh_us, " us",
                    boot.first_refresh_us == 0u ? " (NO END OF REFRESH)" : "", brio::crlf);
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
