// test_stm32f4_dsi - the reference bench suite for the STM32F4's DSI Host:
// the MIPI Display Serial Interface in front of the LTDC, its regulator,
// PLL and D-PHY, the generic interface a panel is spoken to through, and
// video mode out of the external memory - stm32f4/dsi.hpp, with
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
// controller reports its identity, its power mode, its pixel format and
// its own count of link errors over the same link that drives it. That
// is what makes most of this suite a measurement with no eyes: the
// verdicts are the panel's answers, and the picture is for a human only
// in the last letter.
//
//   the panel   the MB1166's controller, identified at boot by RDID1/2/3
//               (DAh, DBh, DCh) against the two the module was built
//               with - Orise's OTM8009A (ID1 40h, ID2 00h, ID3 00h) and
//               Novatek's NT35510 (00h, 80h, 00h) - and driven in DSI
//               video mode with the standard DCS set alone (SLPOUT,
//               COLMOD, MADCTR, CASET/PASET, DISPON, the brightness pair,
//               RAMWR, TEON); reset on PH7 (shared with the touch
//               controller), tearing effect on PJ2 (AF13 to the wrapper,
//               EXTI line 2 to the core), the backlight's boost converter
//               enabled by the panel's own CABC output, so WRCTRLD's BL
//               bit is the backlight switch (UM1932 4.14, R117 fitted)
//   the link    two lanes at 496 Mbit/s: the DSI PLL off the 8 MHz crystal
//               with IDF 1, NDIV 62, ODF 1 (the VCO at 992 MHz - 500 is not
//               exact with the PFD held inside both documents' ranges),
//               the lane byte clock at 62 MHz, the TX escape clock at
//               15.5 MHz, the D-PHY timings from DS11189 table 46
//   the frame   800x480 at a 26.4 MHz pixel clock (PLLSAI N 132, R 5, the
//               LCD divider 2 over the main PLL's M of 4): HSA 2, HBP 20,
//               HFP 20 pixels, VSA 10, VBP 15, VFP 16 lines - the panel
//               datasheet's typical porches - 60.2 frames a second,
//               non-burst with sync pulses, 24-bit pixels on the link
//   the memory  the 128-Mbit SDRAM on FMC bank 1 at 0xC0000000, 32 bits
//               wide, brought up by stm32f4/fmc.hpp before a pixel is
//               fetched; the frame buffer 16-bit pixels
//   the clocks  the core at 180 MHz, the memory at 90 MHz, PCLK2 at 90 MHz
//
// What is exercised, letter by letter:
//   a  the block: the gate closed at reset and the registers behind it,
//      the configuration read back, the vector, the lane byte clock's source
//   b  the regulator and the PLL: the PLL dropped and relocked with its lock
//      time, its unlock flag, the regulator dropped and back, the panel
//      still answering after the link came back
//   c  what the driver refuses, and that a refusal writes nothing
//   d  the D-PHY and the host at rest: the lanes in their stop state, the
//      FIFOs empty, no error standing after the boot's traffic
//   e  THE PANEL'S IDENTITY OVER THE LINK: the three ID bytes, the
//      controller named, the reads repeatable, the panel's own DSI error
//      count at zero
//   f  the panel's registers written and read back: pixel format, address
//      mode, tearing effect, brightness, the backlight control, display on
//      and off, sleep in and out - each through its read-back command
//   g  the errors and the interrupt: a forced error read and cleared by the
//      read, the vector entered once, the wrapper's flags
//   h  the pattern generator: the host's own colour bars with the LTDC
//      stopped, judged by the error registers and the panel
//   i  VIDEO MODE OUT OF THE EXTERNAL MEMORY: the LTDC's frame over the
//      link, the frame rate measured, no payload write error, the panel's
//      error count still zero, a read answered while the video runs
//   j  the tearing effect line: the panel's pulses counted on EXTI line 2
//      and by the wrapper from the pin
//   k  the wrapper's shutdown and colour mode packets, against the panel's
//      power mode
//   l  the ultra-low-power state on the data lanes, entered and left
//   m  colour bars and a moving bar, for a human to look at
//
// build: boards = f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma2d.hpp"
#include "stm32f4/dsi.hpp"
#include "stm32f4/exti.hpp"
#include "stm32f4/fmc.hpp"
#include "stm32f4/ltdc.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
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
/// LTDC's business only: on this board they never reach a pad, and the
/// wrapper is told the same values.
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
constexpr DsiConfig dsi_cfg = dsi_config_for(SysClock::root_hz, bit_rate_hz);
static_assert(dsi_cfg.pll.ndiv == 62 && dsi_cfg.pll.idf == 1 && dsi_cfg.pll.odf == 1);
constexpr uint32_t lane_byte_hz = dsi_lane_byte_hz(bit_rate_hz);
constexpr DsiVideoTiming link_timing = dsi_video_timing(panel_timing, pixel_hz, lane_byte_hz);
static_assert(dsi_line_fits(link_timing, DsiColor::rgb888, DsiLanes::two));
constexpr DsiVideoConfig video_cfg{};   // non-burst, sync pulses, every return to LP allowed

// ---- the pads that are the panel's -------------------------------------------------

using PanelReset = Pin<'H', 7>;   // shared with the touch controller (UM1932 4.14)
using PanelTe = Pin<'J', 2>;      // DSIHOST_TE on AF13 (DS11189 table 12), EXTI line 2
using TeInt = ExtInt<PanelTe>;

// ---- the frame buffers --------------------------------------------------------------

constexpr uint32_t sdram_base = Sdram::base;
constexpr uint32_t fb16_offset = 0x0000'0000u;   // 768 KB
constexpr uint32_t fb32_offset = 0x0010'0000u;   // 1.5 MB
const LtdcFramebuffer<uint16_t> fb16{Sdram::at<uint16_t>(fb16_offset), panel_width, panel_height, 0};
const LtdcFramebuffer<uint32_t> fb32{Sdram::at<uint32_t>(fb32_offset), panel_width, panel_height, 0};

// ---- the panel's command set: DCS, as both datasheets spell it ------------------------

constexpr uint8_t dcs_swreset = 0x01;
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
constexpr uint8_t dcs_paset = 0x2B;
constexpr uint8_t dcs_ramwr = 0x2C;
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
    bool colour_ok = false, video_ok = false;
    uint32_t regulator_us = 0, lock_us = 0;
    uint32_t e0_link = 0, e0_reset = 0, e0_ids = 0, e0_init = 0, e0_paint = 0;
    uint8_t panel = unknown_panel;
    uint8_t id[3] = {0, 0, 0};
    DsiStatus id_status = DsiStatus::refused;
};
BootState boot;

// ---- the events ------------------------------------------------------------------------

volatile uint32_t line_events = 0;
volatile uint32_t dsi_irq_entries = 0;
volatile uint32_t dsi_irq_errors0 = 0;
volatile uint32_t dsi_irq_errors1 = 0;
volatile uint32_t dsi_irq_wrapper = 0;
volatile uint32_t te_edges = 0;

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

/// The standard DCS bring-up, the same for both controllers: out of
/// sleep, the pixel format, the landscape address mode over the whole
/// 800x480 window, the display on, the backlight through the brightness
/// block, and the memory write that video mode's pixels land in.
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
        const uint8_t columns[4] = {0x00, 0x00, static_cast<uint8_t>((panel_width - 1u) >> 8),
                                    static_cast<uint8_t>((panel_width - 1u) & 0xFFu)};
        st = Dsi::dcs_write(dcs_caset, columns, 4);
    }
    if (st == DsiStatus::ok) {
        const uint8_t pages[4] = {0x00, 0x00, static_cast<uint8_t>((panel_height - 1u) >> 8),
                                  static_cast<uint8_t>((panel_height - 1u) & 0xFFu)};
        st = Dsi::dcs_write(dcs_paset, pages, 4);
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
    if (st == DsiStatus::ok) {
        st = Dsi::dcs_write(dcs_ramwr);
    }
    wait_ms(20);
    return st;
}

// =============================================================================
// The display tier, brought up in 18.14.1's order
// =============================================================================

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

constexpr uint16_t blanking_line = static_cast<uint16_t>(panel_timing.vsync + panel_timing.vbp + panel_timing.height);

/// The LTDC showing the 16-bit surface, its line event armed on the first
/// blanking line, running.
bool ltdc_start() {
    LtdcLayer<2>::disable();
    const bool ok = LtdcLayer<1>::configure(one_layer(), panel_timing);
    LtdcLayer<1>::enable();
    (void)Ltdc::line_interrupt(blanking_line);
    Ltdc::reload(LtdcReload::immediate);
    Ltdc::interrupt(LtdcEvent::line, true);
    Ltdc::enable_interrupt();
    Ltdc::enable();
    return ok;
}

/// The LTDC stopped AT THE FRAME BOUNDARY, so that no pixel packet is cut
/// short: the line event is armed on the first blanking line, and the
/// disable right after it lands inside the vertical front porch. A
/// precaution and not a measured need - the invalid transmission length
/// the panel reports (letter d) comes from the host's disable with no
/// stream, and stopping mid-frame was not tried.
void ltdc_stop() {
    if (Ltdc::enabled()) {
        const uint32_t target = line_events + 1u;
        const uint32_t deadline = millis() + 60u;
        while (line_events < target && static_cast<int32_t>(millis() - deadline) < 0) {
        }
    }
    Ltdc::disable();
    Ltdc::interrupt(LtdcEvent::line, false);
    Ltdc::disable_interrupt();
}

/// After the host's enable: two frames for the stream, then the first
/// read, which carries the acknowledge the panel sends for the lanes'
/// transition (AE6, a false control error, at the first bus turnaround
/// after a reset on either side - measured), and the error register
/// read clear.
void link_settle() {
    wait_ms(40);
    uint8_t b = 0;
    (void)Dsi::dcs_read(dcs_rdid1, &b, 1);
    (void)Dsi::errors0();
    (void)Dsi::errors1();
}

/// Wait for `frames` line events; false if none come.
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

/// Eight vertical bars of the primaries into the 16-bit surface, by the
/// accelerator.
bool paint_bars() {
    constexpr uint32_t bars[8] = {argb8888(255, 255, 255, 255), argb8888(255, 255, 255, 0),
                                  argb8888(255, 0, 255, 255),   argb8888(255, 0, 255, 0),
                                  argb8888(255, 255, 0, 255),   argb8888(255, 255, 0, 0),
                                  argb8888(255, 0, 0, 255),     argb8888(255, 0, 0, 0)};
    const uint16_t bar_width = panel_width / 8u;
    bool painted = true;
    for (uint32_t i = 0; i < 8u; ++i) {
        const Dma2dOutput out{.address = sdram_base + fb16_offset + 2u * (i * bar_width),
                              .line_offset = static_cast<uint16_t>(panel_width - bar_width),
                              .format = Dma2dOutputColor::rgb565};
        painted = painted && Dma2d::fill(out, bars[i], Dma2dArea{.pixels = bar_width, .lines = panel_height}) &&
                  Dma2d::wait();
    }
    return painted;
}

/// The link brought up: the regulator, the PLL, the D-PHY, the host, the
/// LTDC interface in 24 bits, video mode with the frame above, the
/// tearing effect from the pin, the host enabled. Times the two waits.
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
    boot.video_ok = Dsi::video(video_cfg, link_timing);
    (void)Dsi::te_source(true, false);
    Dsi::enable();
    return boot.colour_ok && boot.video_ok;
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
          ", commands in ", Dsi::commands_low_power() ? "low power" : "high speed", ", channel ",
          Dsi::virtual_channel(), crlf);
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
    const DsiVideoTiming v = Dsi::video_timing();
    print(serial, "  the frame on the link: HSA ", v.hsa, " HBP ", v.hbp, " line ", v.hline, " lane byte clocks, VSA ",
          v.vsa, " VBP ", v.vbp, " VFP ", v.vfp, " VA ", v.va, " lines, ", v.width, " pixels a packet", crlf);
    bench.verdict("the video registers hold the converted frame (HSA 5, HBP 47, line 1977, 480 lines)",
                  v.hsa == link_timing.hsa && v.hbp == link_timing.hbp && v.hline == link_timing.hline &&
                      v.va == panel_height && v.width == panel_width);
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

    // The host quiet (the LTDC keeps streaming into a wrapper that forwards
    // nothing), the PLL dropped: PLLLS falls. PLLUIF does NOT rise for a
    // stop the program asked for - measured, the flag is a lost lock's.
    const uint32_t pconfr = Dsi::regs().PCONFR;
    const uint32_t cmcr = Dsi::regs().CMCR;
    const uint32_t pcr = Dsi::regs().PCR;
    const uint32_t lcolcr = Dsi::regs().LCOLCR;
    const uint32_t vlcr = Dsi::regs().VLCR;
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
                      Dsi::regs().LCOLCR == lcolcr && Dsi::regs().VLCR == vlcr && Dsi::regs().CLTCR == cltcr;
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
    const bool phy_r = !Dsi::phy(dsi_cfg.phy, dsi_cfg.bit_rate_hz);
    const bool host_r = !Dsi::host(dsi_cfg.host);
    const bool video_r = !Dsi::video(video_cfg, link_timing);
    const bool colour_r = !Dsi::colour(DsiColor::rgb565_1, panel_timing);
    const bool cmd_r = !Dsi::command_mode(panel_width);
    const bool te_r = !Dsi::te_source(false);
    bench.verdict("phy, host, video, colour, command_mode and te_source all refuse while the host is enabled",
                  Dsi::enabled() && phy_r && host_r && video_r && colour_r && cmd_r && te_r);
    bench.verdict("... and none of them wrote a register",
                  Dsi::regs().PCONFR == pconfr && Dsi::regs().CCR == ccr && Dsi::regs().VLCR == vlcr &&
                      Dsi::regs().LCOLCR == lcolcr);

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
    ltdc_stop();
    wait_ms(30);
    const DsiLaneStatus s = Dsi::lane_status();
    print(serial, "  lanes with no video: clock stop ", s.clock_stop ? "1" : "0", " ulps ", s.clock_ulps ? "1" : "0",
          "; data0 stop ", s.data0_stop ? "1" : "0", " ulps ", s.data0_ulps ? "1" : "0", " rx-ulps ",
          s.data0_rx_ulps ? "1" : "0", "; data1 stop ", s.data1_stop ? "1" : "0", " ulps ", s.data1_ulps ? "1" : "0",
          "; direction ", s.direction_rx ? "rx" : "tx", crlf);
    bench.verdict("both data lanes sit in their stop state, in ULPS neither, the bus ours",
                  s.data0_stop && s.data1_stop && !s.data0_ulps && !s.data1_ulps && !s.direction_rx);
    bench.verdict("the clock lane is NOT in stop state: DPCC keeps it in high speed", !s.clock_stop);
    print(serial, "  FIFOs: command ", Dsi::command_fifo_empty() ? "empty" : "NOT EMPTY", ", write ",
          Dsi::write_fifo_empty() ? "empty" : "NOT EMPTY", ", read ", Dsi::read_fifo_empty() ? "empty" : "NOT EMPTY",
          ", read busy ", Dsi::read_busy() ? "YES" : "no", crlf);
    bench.verdict("the three FIFOs are empty and no read is in flight",
                  Dsi::command_fifo_empty() && Dsi::write_fifo_empty() && Dsi::read_fifo_empty() && !Dsi::read_busy());
    const uint32_t e0 = Dsi::errors0();
    const uint32_t e1 = Dsi::errors1();
    print(serial, "  ISR0 along the bring-up: after the link ", hex(boot.e0_link), ", the panel's reset ", hex(boot.e0_reset),
          ", the identity ", hex(boot.e0_ids), ", the init ", hex(boot.e0_init), ", the first frames ", hex(boot.e0_paint),
          "; now ISR0 ", hex(e0), " ISR1 ", hex(e1), crlf);
    bench.verdict("the first bus turnaround after the panel's reset carries AE6, a false control error the "
                  "panel logged while the lanes moved, and nothing else does",
                  boot.e0_link == 0u && boot.e0_reset == 0u && boot.e0_ids == DSI_ISR0_AE6 && boot.e0_init == 0u &&
                      boot.e0_paint == 0u);
    bench.verdict("no error stands now, after the letters so far", e0 == 0u && e1 == 0u);

    // THE GENERIC INTERFACE IN VIDEO MODE WAITS FOR THE STREAM: with the
    // LTDC stopped a read runs out and its command sits in the FIFO;
    // started again, the command goes out with the first frame and is
    // answered. In command mode the same read needs no stream at all.
    uint8_t id = 0xEE;
    const DsiStatus stopped = Dsi::dcs_read(dcs_rdid1, &id, 1);
    const uint32_t gpsr_after = Dsi::regs().GPSR;
    wait_ms(40);
    const uint32_t gpsr_later = Dsi::regs().GPSR;
    (void)ltdc_start();
    wait_ms(40);
    const uint32_t gpsr_running = Dsi::regs().GPSR;
    id = 0xEE;
    const DsiStatus running = Dsi::dcs_read(dcs_rdid1, &id, 1);
    print(serial, "  video mode, LTDC stopped: RDID1 ", status_name(stopped), ", GPSR ", hex(gpsr_after), " then ",
          hex(gpsr_later), "; LTDC started: GPSR ", hex(gpsr_running), ", RDID1 ", hex(id), " (", status_name(running),
          ")", crlf);
    bench.verdict("in video mode with the LTDC stopped a read runs out unanswered", stopped == DsiStatus::timeout);
    bench.verdict("with the stream back the same read is answered", running == DsiStatus::ok && id == boot.id[0]);
    ltdc_stop();
    wait_ms(30);
    Dsi::disable();
    (void)Dsi::command_mode(panel_width);
    Dsi::enable();
    id = 0xEE;
    const DsiStatus in_command = Dsi::dcs_read(dcs_rdid1, &id, 1);
    const uint32_t e0_cmd = Dsi::errors0();
    Dsi::disable();
    (void)Dsi::video(video_cfg, link_timing);
    Dsi::enable();
    print(serial, "  command mode, LTDC stopped: RDID1 ", hex(id), " (", status_name(in_command), "), ISR0 after it ",
          hex(e0_cmd), crlf);
    bench.verdict("in command mode the read is answered with no stream at all",
                  in_command == DsiStatus::ok && id == boot.id[0]);
    bench.verdict("the host's disable with no stream running is what the panel reports at that turnaround: an "
                  "invalid transmission length (AE13), and nothing else",
                  e0_cmd == DSI_ISR0_AE13);

    // With the video running the clock lane is in high speed as well, and
    // the data lanes leave stop state for each burst - a snapshot may
    // catch either, so only the errors are judged.
    (void)ltdc_start();
    link_settle();
    (void)wait_frames(3);
    const DsiLaneStatus v = Dsi::lane_status();
    print(serial, "  lanes with video: clock stop ", v.clock_stop ? "1" : "0", ", data0 stop ", v.data0_stop ? "1" : "0",
          ", data1 stop ", v.data1_stop ? "1" : "0", crlf);
    const uint32_t e0v = Dsi::errors0();
    const uint32_t e1v = Dsi::errors1();
    print(serial, "  ISR0 ", hex(e0v), " ISR1 ", hex(e1v), crlf);
    bench.verdict("three frames in, no error stands either", e0v == 0u && e1v == 0u);
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

    // The brightness and the control block.
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
    // A sleep may take the display with it: the bring-up's tail again.
    (void)Dsi::dcs_write(dcs_dispon);
    (void)Dsi::dcs_write(dcs_ramwr);
    wait_ms(20);
    const uint8_t errors = panel_read(dcs_rdnumed);
    bench.verdict("after all of the above the panel's DSI error count is still zero", errors == 0u);
    bench.verdict("... and the host's error registers too", Dsi::errors0() == 0u && Dsi::errors1() == 0u);
}

// =============================================================================
// g - the errors and the interrupt
// =============================================================================

void tg_errors() {
    // A forced acknowledge error stands in ISR0, and the read that shows it
    // clears it (18.13.2).
    // (The store into FIR takes a moment to show in ISR: read straight
    // after it, ISR0 was still 0 and the bit came with the next read.)
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
}

// =============================================================================
// h - the pattern generator
// =============================================================================

void th_pattern() {
    ltdc_stop();
    wait_ms(30);
    (void)Dsi::errors1();
    Dsi::pattern(DsiPattern::vertical_bars);
    wait_ms(1500);
    const uint32_t e1v = Dsi::errors1();
    const uint8_t pm_v = panel_read(dcs_rddpm);
    Dsi::pattern(DsiPattern::horizontal_bars);
    wait_ms(1500);
    const uint32_t e1h = Dsi::errors1();
    Dsi::pattern(DsiPattern::off);
    (void)ltdc_start();
    wait_ms(40);
    const uint8_t errors = panel_read(dcs_rdnumed);
    print(serial, "  vertical bars 1.5 s: ISR1 ", hex(e1v), ", panel power mode ", hex(pm_v), "; horizontal bars 1.5 s: ISR1 ",
          hex(e1h), "; the panel's DSI error count after ", errors, crlf);
    bench.verdict("the host streams its own colour bars with the LTDC stopped and raises no error",
                  Dsi::pattern_on() == false && e1v == 0u && e1h == 0u);
    bench.verdict("the panel stays on through both patterns and counts no link error",
                  (pm_v & rddpm_dison) != 0u && errors == 0u);
}

// =============================================================================
// i - video mode out of the external memory
// =============================================================================

void ti_video() {
    (void)paint_bars();
    (void)ltdc_start();
    (void)Dsi::errors1();
    Ltdc::clear(ltdc_error_events);
    line_events = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 1000u) {
    }
    const uint32_t frames = line_events;
    const uint32_t e1 = Dsi::errors1();
    const bool underrun = Ltdc::flag(LtdcEvent::fifo_underrun);
    print(serial, "  one second of video: ", frames, " frames (", frame_rate_mhz / 1000u, ".", (frame_rate_mhz / 100u) % 10u,
          " expected), ISR1 ", hex(e1), ", LTDC underrun ", underrun ? "YES" : "no", crlf);
    bench.verdict("the frame rate over the link is the pixel clock's: 60 frames a second, within one",
                  frames + 1u >= frame_rate_mhz / 1000u && frames <= frame_rate_mhz / 1000u + 1u);
    bench.verdict("no LTDC payload write error and no FIFO underrun over a second of frames",
                  (e1 & DSI_ISR1_LPWRE) == 0u && !underrun);

    // A read while the video runs: the command goes out in low power in a
    // blanking period (VMCR.LPCE), and the answer comes back the same way.
    uint8_t id = 0;
    const DsiStatus st = Dsi::dcs_read(dcs_rdid1, &id, 1);
    DsiStatus s_err = DsiStatus::ok;
    const uint8_t errors = panel_read(dcs_rdnumed, s_err);
    print(serial, "  during the video: RDID1 ", hex(id), " (", status_name(st), "), the panel's DSI error count ", errors, " (",
          status_name(s_err), ")", crlf);
    bench.verdict("a DCS read is answered while the video streams", st == DsiStatus::ok && id == boot.id[0]);
    bench.verdict("the panel counted no link error over the video", s_err == DsiStatus::ok && errors == 0u);
    bench.verdict("the host's error registers are clear after it", Dsi::errors0() == 0u && Dsi::errors1() == 0u);

    // Thirty-two bits a pixel from the same memory: twice the fetch, the
    // same link.
    fb32.fill(argb8888(255, 0, 96, 160));
    LtdcLayerConfig c = one_layer();
    c.format = LtdcPixelFormat::argb8888;
    c.framebuffer = sdram_base + fb32_offset;
    (void)LtdcLayer<1>::configure(c, panel_timing);
    Ltdc::reload(LtdcReload::vertical_blanking);
    (void)wait_frames(3);
    Ltdc::clear(ltdc_error_events);
    (void)Dsi::errors1();
    line_events = 0;
    (void)wait_frames(60);
    const uint32_t frames32 = line_events;
    const bool underrun32 = Ltdc::flag(LtdcEvent::fifo_underrun);
    const uint32_t e132 = Dsi::errors1();
    print(serial, "  32-bit pixels: ", frames32, " frames, ISR1 ", hex(e132), ", underrun ", underrun32 ? "YES" : "no", crlf);
    bench.verdict("a 32-bit frame buffer out of the same memory streams as well, no underrun, no payload error",
                  frames32 >= 60u && !underrun32 && (e132 & DSI_ISR1_LPWRE) == 0u);
    (void)LtdcLayer<1>::configure(one_layer(), panel_timing);
    Ltdc::reload(LtdcReload::vertical_blanking);
    (void)wait_frames(2);
}

// =============================================================================
// j - the tearing effect line
// =============================================================================

void tj_tearing() {
    // The panel's TE output on PJ2: counted on EXTI line 2 for half a
    // second with the video running, and by the wrapper from the same pin.
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
    const uint32_t t0 = millis();
    while (millis() - t0 < 500u) {
        if ((Dsi::wrapper_flags() & DSI_WISR_TEIF) != 0u) {
            te_flags = te_flags + 1u;
            Dsi::clear_wrapper_flags(DSI_WISR_TEIF);
        }
    }
    Nvic::disable(TeInt::irq());
    (void)TeInt::arm(false);
    const uint32_t edges = te_edges;
    print(serial, "  TEON, half a second: ", edges, " rising edges on PJ2 (EXTI line 2), ", te_flags,
          " tearing effect flags in the wrapper from the pin", crlf);
    bench.verdict("the panel pulses its TE line once a frame: 25..35 edges in half a second", edges >= 25u && edges <= 35u);
    bench.verdict("the wrapper sees the same pulses from the pin (WCFGR.TESRC), within two",
                  te_flags + 2u >= edges && te_flags <= edges + 2u);

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
    ltdc_stop();
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
    (void)ltdc_start();
    wait_ms(40);
    uint8_t id = 0;
    const DsiStatus st = Dsi::dcs_read(dcs_rdid1, &id, 1);
    bench.verdict("the panel answers after the excursion", st == DsiStatus::ok && id == boot.id[0]);
}

// =============================================================================
// m - a picture for a human
// =============================================================================

void tm_picture() {
    const bool painted = paint_bars();
    (void)ltdc_start();
    bench.verdict("eight colour bars painted by the accelerator", painted);
    print(serial, "  eight bars, white to black, for three seconds; then a bar slides along the bottom", crlf);
    wait_ms(1500);
    const uint16_t strip_y = static_cast<uint16_t>(panel_height - 40u);
    const Dma2dOutput strip{.address = sdram_base + fb16_offset + 2u * strip_y * panel_width,
                            .line_offset = 0,
                            .format = Dma2dOutputColor::rgb565};
    uint32_t steps = 0;
    const uint32_t start = millis();
    uint32_t previous = line_events;
    while (millis() - start < 3000u) {
        if (line_events == previous) {
            continue;
        }
        previous = line_events;
        const uint16_t x = static_cast<uint16_t>((steps * 4u) % (panel_width - 40u));
        (void)Dma2d::fill(strip, argb8888(255, 0, 0, 0), Dma2dArea{.pixels = panel_width, .lines = 40});
        (void)Dma2d::wait();
        const Dma2dOutput bar{.address = strip.address + 2u * x,
                              .line_offset = static_cast<uint16_t>(panel_width - 40u),
                              .format = Dma2dOutputColor::rgb565};
        (void)Dma2d::fill(bar, argb8888(255, 255, 255, 255), Dma2dArea{.pixels = 40, .lines = 40});
        (void)Dma2d::wait();
        ++steps;
    }
    print(serial, "  ", steps, " steps in three seconds", crlf);
    bench.verdict("the bar moved once a frame for three seconds", steps >= 150u);
    (void)paint_bars();
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
    dsi_irq_wrapper = dsi_irq_wrapper | w;
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
    // layer described but nothing enabled, then the link, then the panel,
    // and the LTDC last.
    claim_sdram_pads();
    boot.sdram_up = Sdram::initialize(clock, sdram_geometry, sdram_timing, sdram_boot);
    brio::Ltdc::clock(true);
    boot.pixel_clock_up = brio::Ltdc::pixel_clock(pixel_pll);
    (void)brio::Ltdc::timing(panel_timing);
    brio::Ltdc::background(0, 0, 0);
    brio::Dma2d::init();
    PanelTe::function(brio::PinFunction::af13);
    boot.dsi_up = link_up();
    // THE STREAM FIRST. In video mode the generic interface sends its
    // packets in the blanking periods of a frame, and with the LTDC stopped
    // a command waits in the FIFO for a frame that never comes (letter d
    // measures it): a black frame streams before the panel is spoken to.
    if (boot.sdram_up) {
        fb16.fill(0);
        (void)ltdc_start();
    }
    boot.e0_link = brio::Dsi::errors0();
    panel_reset();
    boot.e0_reset = brio::Dsi::errors0();
    boot.panel = identify(boot.id, boot.id_status);
    boot.e0_ids = brio::Dsi::errors0();
    const DsiStatus panel_st = panel_init();
    boot.e0_init = brio::Dsi::errors0();
    if (boot.sdram_up) {
        (void)paint_bars();
    }
    boot.e0_paint = brio::Dsi::errors0();

    bench.letter('a', "the block", ta_block);
    bench.letter('b', "the regulator and the PLL", tb_regulator_pll);
    bench.letter('c', "what the driver refuses", tc_refusals);
    bench.letter('d', "the D-PHY and the host at rest", td_phy);
    bench.letter('e', "the panel's identity over the link", te_identity);
    bench.letter('f', "the panel's registers written and read back", tf_registers);
    bench.letter('g', "the errors and the interrupt", tg_errors);
    bench.letter('h', "the pattern generator", th_pattern);
    bench.letter('i', "video mode out of the external memory", ti_video);
    bench.letter('j', "the tearing effect line", tj_tearing);
    bench.letter('k', "the wrapper's shutdown and colour mode packets", tk_wrapper_packets);
    bench.letter('l', "the ultra-low-power state on the data lanes", tl_ulps);
    bench.letter('m', "colour bars and a moving bar, for a human to look at", tm_picture);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
                    " sdram=", boot.sdram_up ? "up" : "FAILED", " lcdclk=", boot.pixel_clock_up ? "locked" : "FAILED",
                    " dsi=", boot.dsi_up ? "up" : "FAILED", " panel=", panel_name(boot.panel), " (", brio::hex(boot.id[0]),
                    " ", brio::hex(boot.id[1]), " ", brio::hex(boot.id[2]), ", ids ", status_name(boot.id_status), ", init ",
                    status_name(panel_st), ")", brio::crlf);
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
