// DSI Host family smoke TU (RM0386 ch. 18). THE POINT OF THIS TU IS THE
// ABSENCE AND THE ARITHMETIC: two of the twenty-three headers declare
// DSI_BASE and the resource exists there; the other twenty-one have no
// block, and the vocabulary above it - the PLL solver, the unit
// interval, the lane timings, the frame conversion - compiles and is
// checked on every part alike.
#include "stm32f4/dsi.hpp"

using namespace brio;

// ---- the reserve's presence answers ------------------------------------------------

static_assert(dsi_present() == (dsi_clock_mask() != 0u));
static_assert(dsi_present() == (dsi_reset_mask() != 0u));
static_assert(dsi_present() == (dsi_byte_clock_select_mask() != 0u));
static_assert(!dsi_present() || ltdc_present(), "a DSI host stands in front of an LTDC");

#if defined(STM32F469xx) || defined(STM32F479xx)
static_assert(dsi_present());
static_assert(dsi_clock_mask() == RCC_APB2ENR_DSIEN);
static_assert(dsi_reset_mask() == RCC_APB2RSTR_DSIRST);
static_assert(dsi_byte_clock_select_mask() == RCC_DCKCFGR_DSISEL);
#else
static_assert(!dsi_present());
#endif

// ---- the PLL (18.12.5, DS11189 table 47) -------------------------------------------

// The board's 8 MHz crystal: 496 Mbit/s is exact with the PFD at 8 MHz
// (IDF 1, NDIV 62, ODF 1: the VCO at 992 MHz), 500 is not - it would
// want IDF 2 and a 4 MHz PFD, inside the data sheet's range and outside
// the manual's.
constexpr DsiPllConfig at_496 = dsi_pll_for(8'000'000u, 496'000'000u);
static_assert(at_496.ndiv == 62 && at_496.idf == 1 && at_496.odf == 1);
static_assert(dsi_pll_vco_hz(8'000'000u, at_496) == 992'000'000u);
static_assert(dsi_pll_bit_rate_hz(8'000'000u, at_496) == 496'000'000u);
static_assert(dsi_pll_config_ok(8'000'000u, at_496));
static_assert(dsi_pll_for(8'000'000u, 500'000'000u).ndiv == 0u, "not exact with the PFD held at 8 MHz");
// A 25 MHz crystal reaches 500 exactly: IDF 1, NDIV 20, the VCO at 1 GHz.
constexpr DsiPllConfig at_500 = dsi_pll_for(25'000'000u, 500'000'000u);
static_assert(at_500.ndiv == 20 && at_500.idf == 1 && at_500.odf == 1);
// The output divider: 62 Mbit/s off 8 MHz is the same VCO over ODF 8.
constexpr DsiPllConfig at_62 = dsi_pll_for(8'000'000u, 62'000'000u);
static_assert(at_62.ndiv == 62 && at_62.idf == 1 && at_62.odf == 8);
// Below the output's floor and above its ceiling: nothing.
static_assert(dsi_pll_for(8'000'000u, 30'000'000u).ndiv == 0u);
static_assert(dsi_pll_for(8'000'000u, 600'000'000u).ndiv == 0u);
static_assert(dsi_pll_for(8'000'000u, 0u).ndiv == 0u);
// The rules on a hand-written triple.
static_assert(!dsi_pll_config_ok(8'000'000u, DsiPllConfig{.ndiv = 9, .idf = 1, .odf = 1}), "NDIV is 10..125");
static_assert(!dsi_pll_config_ok(8'000'000u, DsiPllConfig{.ndiv = 126, .idf = 1, .odf = 1}));
static_assert(!dsi_pll_config_ok(8'000'000u, DsiPllConfig{.ndiv = 62, .idf = 0, .odf = 1}), "IDF is 1..7");
static_assert(!dsi_pll_config_ok(8'000'000u, DsiPllConfig{.ndiv = 62, .idf = 1, .odf = 3}), "ODF is 1, 2, 4 or 8");
static_assert(!dsi_pll_config_ok(8'000'000u, DsiPllConfig{.ndiv = 62, .idf = 2, .odf = 1}), "the PFD input under 8 MHz");
static_assert(!dsi_pll_config_ok(8'000'000u, DsiPllConfig{.ndiv = 30, .idf = 1, .odf = 1}), "the VCO under 500 MHz");
static_assert(!dsi_pll_config_ok(3'000'000u, DsiPllConfig{.ndiv = 100, .idf = 1, .odf = 1}), "CLKIN under 4 MHz");
static_assert(!dsi_pll_config_ok(8'000'000u, DsiPllConfig{.ndiv = 62, .idf = 1, .odf = 4}) ==
              !(496'000'000u / 4u >= dsi_pll_out_min_hz));

// ---- the unit interval and the derived clocks --------------------------------------

static_assert(dsi_lane_byte_hz(496'000'000u) == 62'000'000u);
static_assert(dsi_lane_byte_hz(500'000'000u) == 62'500'000u);
static_assert(dsi_uix4(496'000'000u) == 8u, "2.016 ns is 8.06 quarters, rounded down");
static_assert(dsi_uix4(600'000'000u) == 6u, "18.16.6's own example");
static_assert(dsi_uix4(80'000'000u) == 50u);
static_assert(dsi_uix4(0u) == 0u);
static_assert(dsi_escape_divider(62'000'000u) == 4u, "15.5 MHz, under the 20 MHz ceiling");
static_assert(dsi_escape_divider(62'500'000u) == 4u);
static_assert(dsi_escape_divider(20'000'000u) == 2u, "never below 2: 0 and 1 stop the clock");
static_assert(dsi_cycles_for_ns(62'000'000u, 1000u) == 62u);
static_assert(dsi_cycles_for_ns(62'000'000u, 1u) == 1u, "rounded up");

// ---- the lane transition times (DS11189 table 46) ----------------------------------

constexpr DsiPhyTiming phy_496 = dsi_phy_timing_for(496'000'000u);
// Clock lane LP to HS: 50 + 300 + 8 UI (17 ns) = 367 ns, half again 550, at
// 16.13 ns a cycle 35; HS to LP: 62 + 52 UI (105) + 60 + 100 = 327, 490,
// 31. Data lane LP to HS: 50 + 145 + 10 UI (21) = 216, 324, 21; HS to LP:
// 60 + 8 UI (17) + 100 = 177, 265, 17.
static_assert(phy_496.clock_lp2hs == 35 && phy_496.clock_hs2lp == 31);
static_assert(phy_496.data_lp2hs == 21 && phy_496.data_hs2lp == 17);
static_assert(phy_496.max_read_time == 0x7FFF && phy_496.stop_wait == 10);
static_assert(dsi_phy_timing_ok(phy_496));
static_assert(!dsi_phy_timing_ok(DsiPhyTiming{.clock_hs2lp = 1024}), "ten bits");
static_assert(!dsi_phy_timing_ok(DsiPhyTiming{.max_read_time = 0x8000}), "fifteen bits");
static_assert(dsi_phy_timing_for(0u).clock_lp2hs == 0u);

// ---- the frame converted (18.14.6) -----------------------------------------------

// A 800x480 panel: HSA 2, HBP 20, HFP 20 pixels, VSA 10, VBP 15, VFP 16
// lines, at a 26.4 MHz pixel clock over a 62 MHz lane byte clock (2.348
// cycles a pixel): HSA 5, HBP 47, the 842-pixel line 1977.
constexpr LtdcTiming panel{.hsync = 2, .hbp = 20, .width = 800, .hfp = 20,
                           .vsync = 10, .vbp = 15, .height = 480, .vfp = 16};
constexpr DsiVideoTiming frame = dsi_video_timing(panel, 26'400'000u, 62'000'000u);
static_assert(frame.hsa == 5 && frame.hbp == 47 && frame.hline == 1977);
static_assert(frame.vsa == 10 && frame.vbp == 15 && frame.vfp == 16 && frame.va == 480 && frame.width == 800);
static_assert(dsi_video_timing_ok(frame));
static_assert(dsi_lane_cycles(1, 26'400'000u, 62'000'000u) == 2u, "2.348 rounds to 2");
static_assert(dsi_lane_cycles(2, 26'400'000u, 62'000'000u) == 5u, "4.697 rounds to 5");
static_assert(dsi_lane_cycles(1, 0u, 62'000'000u) == 0u);
// 800 pixels of 24 bits are 2400 bytes plus the packet's 6, 1203 lane byte
// clocks a lane on two lanes: it fits a 1977-cycle line with its 52 of
// sync and porch, and would not on one lane at a line half as long.
static_assert(dsi_line_payload_bytes(800, DsiColor::rgb888) == 2406u);
static_assert(dsi_line_payload_bytes(800, DsiColor::rgb565_1) == 1606u);
static_assert(dsi_line_payload_bytes(800, DsiColor::rgb666_1) == 1806u);
static_assert(dsi_line_fits(frame, DsiColor::rgb888, DsiLanes::two));
static_assert(!dsi_line_fits(DsiVideoTiming{.hsa = 5, .hbp = 47, .hline = 1000, .va = 480, .width = 800},
                             DsiColor::rgb888, DsiLanes::one));
// The fields' widths.
static_assert(!dsi_video_timing_ok(DsiVideoTiming{.hsa = 4096, .hline = 5000, .width = 8}), "HSA is twelve bits");
static_assert(!dsi_video_timing_ok(DsiVideoTiming{.hline = 32768, .width = 8}), "HLINE is fifteen bits");
static_assert(!dsi_video_timing_ok(DsiVideoTiming{.hline = 100, .vsa = 1024, .width = 8}), "VSA is ten bits");
static_assert(!dsi_video_timing_ok(DsiVideoTiming{.hline = 100, .va = 16384, .width = 8}), "VA is fourteen bits");
static_assert(!dsi_video_timing_ok(DsiVideoTiming{.hline = 100, .width = 0}), "a line has pixels");
static_assert(!dsi_video_timing_ok(DsiVideoTiming{.hsa = 60, .hbp = 60, .hline = 100, .width = 8}),
              "the line holds its sync and porch");

// ---- the whole configuration -----------------------------------------------------

constexpr DsiConfig cfg = dsi_config_for(8'000'000u, 496'000'000u);
static_assert(cfg.pll.ndiv == 62 && cfg.bit_rate_hz == 496'000'000u);
static_assert(cfg.phy.timing.clock_lp2hs == 35 && cfg.host.escape_divider == 4);
static_assert(dsi_config_ok(8'000'000u, cfg));
static_assert(!dsi_config_ok(8'000'000u, DsiConfig{.pll = at_496, .bit_rate_hz = 500'000'000u}),
              "the rate stated must be the triple's");
static_assert(!dsi_config_ok(8'000'000u, DsiConfig{.pll = at_496, .bit_rate_hz = 496'000'000u,
                                                   .host = DsiHostConfig{.escape_divider = 1}}),
              "an escape divider of 1 stops the clock");
static_assert(!dsi_host_config_ok(DsiHostConfig{.virtual_channel = 4}), "two bits of channel");
static_assert(dsi_config_for(8'000'000u, 500'000'000u).pll.ndiv == 0u);

#if defined(DSI_BASE)

static_assert(dsi_present());
static_assert(Dsi::irq == dsi_irq());
static_assert(Dsi::odf_code(1) == 0 && Dsi::odf_code(2) == 1 && Dsi::odf_code(4) == 2 && Dsi::odf_code(8) == 3);
static_assert(Dsi::command_type_bits == 0x010F7F00u,
              "GSW0..2TX, GSR0..2TX, GLWTX (8..14), DSW0..1TX, DSR0TX, DLWTX (16..19), MRDPS (24)");

// ---- every verb, once ---------------------------------------------------------------

void every_verb(uint8_t* buf) {
    (void)Dsi::init<cfg, 8'000'000u>();
    (void)Dsi::init(8'000'000u, cfg);
    (void)Dsi::clock();
    (void)Dsi::version();
    (void)Dsi::byte_clock_from_pllr();
    (void)Dsi::regulator(true);
    (void)Dsi::regulator_ready();
    (void)Dsi::pll(8'000'000u, cfg.pll);
    (void)Dsi::pll_on();
    (void)Dsi::pll_locked();
    (void)Dsi::pll_config();
    (void)Dsi::phy(cfg.phy, cfg.bit_rate_hz);
    (void)Dsi::uix4();
    (void)Dsi::lanes();
    (void)Dsi::phy_timing();
    (void)Dsi::lane_status();
    Dsi::ulps(true, true, true);
    (void)Dsi::host(cfg.host);
    (void)Dsi::escape_divider();
    (void)Dsi::commands_low_power();
    (void)Dsi::virtual_channel();
    (void)Dsi::colour(DsiColor::rgb888, panel);
    (void)Dsi::colour();
    (void)Dsi::te_source(true);
    (void)Dsi::adapted_refresh(false, false);
    (void)Dsi::command_mode(800);
    (void)Dsi::video(DsiVideoConfig{}, frame);
    (void)Dsi::in_command_mode();
    (void)Dsi::video_timing();
    Dsi::pattern(DsiPattern::vertical_bars);
    (void)Dsi::pattern_on();
    (void)Dsi::shadow_update();
    (void)Dsi::shadowed();
    Dsi::shadow(false);
    Dsi::enable();
    (void)Dsi::enabled();
    Dsi::ltdc_flow(true);
    (void)Dsi::ltdc_flow();
    Dsi::shutdown(false);
    (void)Dsi::shutdown();
    Dsi::eight_colours(false);
    (void)Dsi::eight_colours();
    (void)Dsi::busy();
    (void)Dsi::short_write(DsiDataType::dcs_short_write_0, 0x11, 0);
    (void)Dsi::long_write(DsiDataType::dcs_long_write, buf, 5);
    (void)Dsi::read(DsiDataType::dcs_read, 0xDA, 0, buf, 1);
    (void)Dsi::dcs_write(0x11);
    (void)Dsi::dcs_write(0x3A, 0x77);
    (void)Dsi::dcs_write(0x2A, buf, 4);
    (void)Dsi::dcs_read(0xDA, buf, 1);
    (void)Dsi::generic_write(buf, 3);
    (void)Dsi::generic_read(buf, 1, buf, 2);
    (void)Dsi::command_fifo_empty();
    (void)Dsi::write_fifo_empty();
    (void)Dsi::read_fifo_empty();
    (void)Dsi::read_busy();
    (void)Dsi::errors0();
    (void)Dsi::errors1();
    Dsi::error_interrupts(DSI_IER0_AE0IE, DSI_IER1_TOHSTXIE);
    Dsi::force_errors(DSI_FIR0_FAE0, 0u);
    Dsi::wrapper_interrupts(DSI_WISR_TEIF | DSI_WISR_PLLLIF);
    (void)Dsi::wrapper_flags();
    Dsi::clear_wrapper_flags(DSI_WISR_TEIF);
    (void)Dsi::isr();
    Dsi::enable_interrupt();
    Dsi::disable_interrupt();
    (void)Dsi::header(DsiDataType::dcs_read, 0xDA, 0);
    Dsi::disable();
    Dsi::reset_block();
    Dsi::release();
}

#endif   // DSI_BASE
