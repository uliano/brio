/*
 * dsi.hpp
 *
 * The STM32F4's DSI Host (RM0386 ch. 18) - the MIPI Display Serial
 * Interface in front of the LTDC, on the F469/F479 class alone (the
 * device header says so with DSI_BASE; the other twenty-one headers of
 * the pack declare no block and nothing register-facing in this file
 * exists there): `Dsi`, a MONOSTATE resource, because a part carries
 * one host or none.
 *
 *   using namespace brio;
 *   constexpr DsiConfig cfg{.pll = dsi_pll_for(8'000'000u, 496'000'000u),
 *                           .bit_rate_hz = 496'000'000u};
 *   Dsi::init<cfg, 8'000'000u>();                  // regulator, PLL, D-PHY, host
 *   Dsi::colour(DsiColor::rgb888, panel_timing);   // the LTDC interface
 *   Dsi::video(video_cfg, dsi_video_timing(panel_timing, pixel_hz, lane_byte_hz));
 *   Dsi::enable();                                 // D-PHY, host, wrapper
 *   Dsi::dcs_write(0x11);                          // SLPOUT, in low power
 *   uint8_t id = 0;
 *   Dsi::dcs_read(0xDA, &id, 1);                   // RDID1, the panel's answer
 *   Ltdc::enable();                                // and the pixels stream
 *
 * WHAT THE BLOCK IS. Three things under one register file and one
 * vector: the DSI WRAPPER (the regulator and the PLL that make the D-PHY
 * run, the LTDC's pixel stream turned into the host's input, the tearing
 * effect line), the DSI HOST (Synopsys' MIPI DSI controller: the packet
 * handler, the video mode state machine, the APB generic interface a
 * program writes DCS and generic packets through, the error registers)
 * and the D-PHY itself (one clock lane, two data lanes, each in
 * high-speed differential or low-power single-ended signalling). Two
 * ways to a panel: VIDEO MODE, where the LTDC's frame is packed into
 * pixel packets and streamed with the panel's timing and the wrapper
 * has nothing to do but pass it on; and ADAPTED COMMAND MODE, where each
 * LTDC frame becomes DCS memory-write packets a panel with a frame
 * memory takes at its own pace, refreshed on a tearing-effect event.
 * The APB generic interface works beside either: a DCS command goes
 * out in low-power signalling during a blanking period, a read turns
 * the bus around and the answer arrives on data lane 0.
 *
 * THE FACTS THAT SHAPE THE FILE.
 *
 * 1. THE PLL AND THE PHY HAVE RANGES THAT TWO DOCUMENTS STATE
 *    DIFFERENTLY. RM0386 18.12.5 gives the PLL's input after IDF as
 *    8..50 MHz; DS11189 table 47 gives the same PFD input as 4..25 MHz.
 *    `dsi_pll_for()` keeps to the INTERSECTION, 8..25 MHz, and the rest
 *    where the two agree: CLKIN 4..100 MHz, the VCO 500..1000 MHz, the
 *    output 31.25..500 MHz, NDIV 10..125, IDF 1..7, ODF 1/2/4/8. The
 *    output IS the bit rate of one lane (the RCC's figure 17 draws the
 *    500 MHz "high speed clock" whose eighth is the lane byte clock), and
 *    DS11189 table 45's unit interval of 2..12.5 ns is that rate's other
 *    face, 80..500 Mbit/s. The PLL's CLKIN is HSE and nothing else.
 * 2. THE UNIT INTERVAL IS THE ONE FIELD THE D-PHY CANNOT DO WITHOUT
 *    (18.14.2): WPCR0.UIX4, the bit period in quarter nanoseconds,
 *    rounded DOWN - `dsi_uix4()`. Every D-PHY timing the wrapper lets a
 *    program override (WPCR1..4) is derived from it and left at its
 *    default here.
 * 3. THE HOST COUNTS IN LANE BYTE CLOCKS. HSA, HBP and the line in
 *    DSI_VHSACR/VHBPCR/VLCR are the LTDC's pixel counts converted at the
 *    ratio of the lane byte clock to the pixel clock and ROUNDED
 *    (18.14.6), and the rounding accumulates line after line unless the
 *    host returns to low power once a line and resynchronises on the
 *    next sync event - which is why `DsiVideoConfig` enables the
 *    low-power returns by default. `dsi_video_timing()` is that
 *    conversion; the LP/HS transition times of the lanes in DSI_CLTCR
 *    and DSI_DLTCR are lane byte clocks too, computed from DS11189 table
 *    46's maxima by `dsi_phy_timing_for()`.
 * 4. CONFIGURATION HAPPENS WITH THE HOST DISABLED. DSI_CR.EN = 0 holds
 *    the host "under reset" (18.15.2) and the wrapper's WCFGR fields and
 *    WPCR1..4 are documented as writable only with both DSI_CR.EN and
 *    DSI_WCR.DSIEN clear; the programming procedure of 18.14.1 writes
 *    everything first and enables last. Every configuring verb here
 *    REFUSES while `enabled()`, and `disable()` is the way back. What the
 *    generic interface needs (a wait on the command FIFO, a header
 *    write, the payload) works only while enabled.
 * 5. THE HOST'S ERROR REGISTERS CLEAR ON READ (18.13.2): DSI_ISR0 and
 *    DSI_ISR1 are cleared by the read that returns them and the line
 *    falls when every register that raised it has been read. So
 *    `errors0()`/`errors1()` are read-and-clear by the silicon's design,
 *    `isr()` reads both, and a program that wants to keep them keeps the
 *    values. The wrapper's five flags are the other kind, cleared through
 *    DSI_WIFCR.
 * 6. A READ IS A WRITE OF TWO PACKETS AND A TURN-AROUND. The host has no
 *    read verb: a program sets the maximum return packet size (the DSI
 *    data type 0x37, its two bytes the size), sends the read request
 *    (a DCS read or a generic read of zero to two parameters), and the
 *    panel takes the bus (BTA, so DSI_PCR.BTAE must be set) and answers
 *    in low-power signalling on lane 0; GPSR.RCB stands until the whole
 *    response is in the payload read FIFO, and the payload comes out of
 *    DSI_GPDR a word at a time, the header and its checks stripped by
 *    the packet analyser. `read()` is that sequence with a bounded wait.
 * 7. THE DATA TYPE CODES ARE THE MIPI DSI SPECIFICATION'S, NOT THE
 *    MANUAL'S: 18.14 assumes them and prints none, and the two panel
 *    datasheets on the desk draw their table as a picture. `DsiDataType`
 *    spells the ones every DSI peripheral shares (DSI 1.1 table 16 -
 *    the processor-to-peripheral types), and a program never needs to
 *    know them for a DCS command: `dcs_write()` and `dcs_read()` choose
 *    the short or long form from the parameter count.
 * 8. TWO WAYS TO A PANEL, AND THE PANEL DECIDES. Video mode streams
 *    pixel packets; the adapted command mode (18.6) turns each LTDC
 *    frame into DCS memory writes - write_memory_start then
 *    write_memory_continue, LCCR.CMDSIZE pixels a packet - launched by
 *    WCR.LTDCEN (or by a tearing effect pulse with WCFGR.AR) and closed
 *    by the wrapper with ERIF, the LTDC halted on the VSync edge WCFGR.
 *    VSPOL names. A panel whose DSI is a command interface, like the
 *    32F469IDISCOVERY's NT35510 (its datasheet documents RAMWR/RAMWRC
 *    and nothing of a video stream), shows nothing of video mode and
 *    everything of a refresh: measured, a second of video frames leaves
 *    its memory untouched and one refresh puts the frame there, read
 *    back pixel for pixel through RAMRD. The memory writes of a refresh
 *    are DCS long writes and want high speed, so `DsiHostConfig` keeps
 *    the long writes' CMCR bits apart from the short commands'.
 * 9. IN VIDEO MODE THE GENERIC INTERFACE LIVES INSIDE THE STREAM. The
 *    manual's procedure (18.14.1) lists the DCS commands before the
 *    LTDC's enable; measured, a read issued in video mode with the LTDC
 *    stopped never completes - the request leaves the command FIFO,
 *    GPSR.RCB stands, and the answer arrives with the first frame after
 *    the LTDC starts. In command mode the same read is answered with
 *    nothing else on the link. Nothing in this file can hide that: the
 *    mode is the program's.
 * 10. WHAT THE PANEL REPORTS IS THE PANEL'S. The acknowledge-with-error
 *    bits of DSI_ISR0 are a panel's opinion of the packets it received,
 *    delivered at the next bus turnaround - so a read is what makes them
 *    appear, and they describe what happened SINCE the previous read.
 *    Measured on the NT35510: the first turnaround after the host's
 *    disable and enable carries AE6 (a false control error: the lanes
 *    moved while its receiver watched) in command mode and with a video
 *    stream running, AE13 (an invalid transmission length) in video
 *    mode with the LTDC stopped; the first after the panel's own reset
 *    carries AE6 when the host was streaming and nothing when it was
 *    not. One report each, nothing follows, and the registers clear on
 *    the read that shows them: a program that re-enables the host reads
 *    them once after its first exchange.
 * 11. THE PANEL'S RECEIVER LOCKS ONTO THE CLOCK LANE'S ENTRY INTO HIGH
 *    SPEED. Measured: a panel reset with the clock lane already running
 *    in high speed took no high-speed packet afterwards - every frame
 *    refreshed into it was lost, every low-power command and read went
 *    through, and neither side counted an error - until the host was
 *    disabled and enabled again, which restarts the clock lane. So a
 *    program brings the clock lane to high speed (`clock_lane(true)`)
 *    after the panel's reset and its low-power bring-up, or keeps
 *    `DsiPhyConfig::clock_lane_hs` false until then.
 *
 * THE ERRATA (ES0321 Rev 14, 2.8.1 .. 2.8.3), two of them code here:
 *  - 2.8.2, "incorrect calculation of the time to activate the clock
 *    between HS transmissions": with the automatic clock lane control
 *    the host uses twice HS2LP_TIME instead of HS2LP + LP2HS. `phy()`
 *    writes BOTH clock-lane fields with the larger of the two values,
 *    the sheet's workaround, whether or not ACR is on.
 *  - 2.8.3, "the immediate update procedure may fail": setting VSCR.UR
 *    and VSCR.EN in one write may race. `shadow_update()` writes 0 then
 *    0x101 and checks UR auto-cleared, repeating while it has not.
 *  - 2.8.1, "tearing effect parasitic detection": the tearing effect
 *    over the LINK raises TEIF on every acknowledge trigger. Not code:
 *    `te_source()` offers the dedicated pin (WCFGR.TESRC = 1), the
 *    sheet's workaround, and a program that chooses the link is told in
 *    the document what not to arm.
 *
 * THREE SMALL FACTS THE REGISTERS TAUGHT, none of them a workaround:
 * DSI_PSR wakes as 0x1400 and not the manual's 0x1528 (its defined bits
 * all clear with the regulator off, two reserved ones set); clearing
 * WRPCR.PLLEN drops PLLLS and raises NO PLLUIF - the unlock flag is a
 * lost lock's, not a software stop's; and a store into DSI_FIR shows in
 * DSI_ISR a moment later, not on the read straight after it.
 *
 * NOT HERE, BY DESIGN: the panel's own command set beyond the DCS
 * spelling of a write and a read. Which command wakes a panel, sets its
 * pixel format, its write window or its backlight is the panel
 * datasheet's business, spelled where the panel is (the bench suite for
 * the 32F469IDISCOVERY's module). The LTDC's side - the pixel clock, the
 * timing, the layers - is stm32f4/ltdc.hpp's, and this file only reads
 * `LtdcTiming` to convert it.
 *
 * CONCURRENCY. Every configuring verb is a plain store into a register
 * with no set/clear twin and is meant for setup with the host disabled.
 * The generic interface's verbs poll the FIFO flags and are for one
 * context at a time. `isr()` is legal from a handler: the wrapper's
 * clears are write-one stores, the host's are reads.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/ltdc.hpp"
#include "stm32f4/nvic.hpp"

namespace brio {

// =============================================================================
// The vocabulary - on every part, block or no block
// =============================================================================

/// The packet data types a processor sends (MIPI DSI 1.1 table 16, the
/// six-bit DT field of DSI_GHCR). A program using DCS never spells one:
/// `Dsi::dcs_write()` and `Dsi::dcs_read()` choose; they are here for a
/// generic packet and for the wrapper's own traffic to be readable.
enum class DsiDataType : uint8_t {
    vsync_start = 0x01,
    vsync_end = 0x11,
    hsync_start = 0x21,
    hsync_end = 0x31,
    end_of_transmission = 0x08,
    colour_mode_off = 0x02,
    colour_mode_on = 0x12,
    shutdown_peripheral = 0x22,
    turn_on_peripheral = 0x32,
    generic_short_write_0 = 0x03,
    generic_short_write_1 = 0x13,
    generic_short_write_2 = 0x23,
    generic_read_0 = 0x04,
    generic_read_1 = 0x14,
    generic_read_2 = 0x24,
    dcs_short_write_0 = 0x05,
    dcs_short_write_1 = 0x15,
    dcs_read = 0x06,
    set_maximum_return_packet_size = 0x37,
    null_packet = 0x09,
    blanking_packet = 0x19,
    generic_long_write = 0x29,
    dcs_long_write = 0x39,
    packed_pixel_rgb565 = 0x0E,
    packed_pixel_rgb666 = 0x1E,
    loosely_packed_pixel_rgb666 = 0x2E,
    packed_pixel_rgb888 = 0x3E,
};

/// The D-PHY's limits, DS11189 tables 45 and 47: a unit interval of
/// 2..12.5 ns is a lane bit rate of 80..500 Mbit/s, and the lane byte
/// clock - the host's own clock, RCC figure 17 - is an eighth of it.
inline constexpr uint32_t dsi_bit_rate_min_hz = 80'000'000u;
inline constexpr uint32_t dsi_bit_rate_max_hz = 500'000'000u;
inline constexpr uint32_t dsi_lane_byte_max_hz = dsi_bit_rate_max_hz / 8u;
/// The TX escape clock, "around 20 MHz" and no more (18.14.3, figure 17).
inline constexpr uint32_t dsi_escape_clock_max_hz = 20'000'000u;

/// The PLL's ranges: CLKIN, the VCO and the output as RM0386 18.12.5 and
/// DS11189 table 47 agree on them, the PFD input where they do not
/// (8..50 MHz in the manual, 4..25 in the data sheet) as their
/// intersection.
inline constexpr uint32_t dsi_pll_in_min_hz = 4'000'000u;
inline constexpr uint32_t dsi_pll_in_max_hz = 100'000'000u;
inline constexpr uint32_t dsi_pll_pfd_min_hz = 8'000'000u;
inline constexpr uint32_t dsi_pll_pfd_max_hz = 25'000'000u;
inline constexpr uint32_t dsi_pll_vco_min_hz = 500'000'000u;
inline constexpr uint32_t dsi_pll_vco_max_hz = 1'000'000'000u;
inline constexpr uint32_t dsi_pll_out_min_hz = 31'250'000u;
inline constexpr uint32_t dsi_pll_out_max_hz = dsi_bit_rate_max_hz;

/**
 * The PLL's three fields (18.12.5, DSI_WRPCR): FVCO = CLKIN / IDF * 2 *
 * NDIV, and the output - the lane bit rate - FVCO / (2 * ODF). `ndiv ==
 * 0` is "no configuration", what `dsi_pll_for()` answers when no exact
 * triple exists.
 */
struct DsiPllConfig {
    uint8_t ndiv = 0;   ///< 10..125
    uint8_t idf = 1;    ///< 1..7
    uint8_t odf = 1;    ///< 1, 2, 4 or 8
};

constexpr uint64_t dsi_pll_vco_hz(uint32_t clkin_hz, const DsiPllConfig& c) {
    if (c.idf == 0u) {
        return 0u;
    }
    return static_cast<uint64_t>(clkin_hz) * 2u * c.ndiv / c.idf;
}

/// The output - one lane's bit rate - in hertz; 0 for an empty config.
constexpr uint32_t dsi_pll_bit_rate_hz(uint32_t clkin_hz, const DsiPllConfig& c) {
    if (c.odf == 0u) {
        return 0u;
    }
    return static_cast<uint32_t>(dsi_pll_vco_hz(clkin_hz, c) / (2u * c.odf));
}

/// The whole rule set of item 1 on a configuration at a given CLKIN.
constexpr bool dsi_pll_config_ok(uint32_t clkin_hz, const DsiPllConfig& c) {
    if (c.ndiv < 10u || c.ndiv > 125u || c.idf < 1u || c.idf > 7u) {
        return false;
    }
    if (c.odf != 1u && c.odf != 2u && c.odf != 4u && c.odf != 8u) {
        return false;
    }
    if (clkin_hz < dsi_pll_in_min_hz || clkin_hz > dsi_pll_in_max_hz) {
        return false;
    }
    const uint32_t pfd = clkin_hz / c.idf;
    if (pfd < dsi_pll_pfd_min_hz || pfd > dsi_pll_pfd_max_hz) {
        return false;
    }
    const uint64_t vco = dsi_pll_vco_hz(clkin_hz, c);
    if (vco < dsi_pll_vco_min_hz || vco > dsi_pll_vco_max_hz) {
        return false;
    }
    const uint32_t out = dsi_pll_bit_rate_hz(clkin_hz, c);
    return out >= dsi_pll_out_min_hz && out <= dsi_pll_out_max_hz;
}

/// The EXACT triple for a lane bit rate of `bit_rate_hz` off `clkin_hz`
/// (HSE), the smallest IDF first so the PFD runs as high as the range
/// allows. `ndiv == 0` when none exists - the caller then asks for a
/// rate its crystal cannot make, and the driver refuses it rather than
/// running the link a few per cent off what the timings were computed
/// for.
constexpr DsiPllConfig dsi_pll_for(uint32_t clkin_hz, uint32_t bit_rate_hz) {
    if (bit_rate_hz == 0u) {
        return DsiPllConfig{};
    }
    constexpr uint8_t odfs[] = {1, 2, 4, 8};
    for (uint8_t idf = 1; idf <= 7u; ++idf) {
        for (uint8_t odf : odfs) {
            // FVCO = out * 2 * ODF = CLKIN / IDF * 2 * NDIV
            const uint64_t vco = static_cast<uint64_t>(bit_rate_hz) * 2u * odf;
            const uint64_t twice_pfd = static_cast<uint64_t>(clkin_hz) * 2u;
            if ((vco * idf) % twice_pfd != 0u) {
                continue;
            }
            const uint64_t n = (vco * idf) / twice_pfd;
            if (n < 10u || n > 125u) {
                continue;
            }
            const DsiPllConfig c{static_cast<uint8_t>(n), idf, odf};
            if (dsi_pll_config_ok(clkin_hz, c) && dsi_pll_bit_rate_hz(clkin_hz, c) == bit_rate_hz) {
                return c;
            }
        }
    }
    return DsiPllConfig{};
}

/// The lane byte clock: the bit rate over eight (figure 17).
constexpr uint32_t dsi_lane_byte_hz(uint32_t bit_rate_hz) { return bit_rate_hz / 8u; }

/// WPCR0.UIX4: the unit interval in quarter nanoseconds, rounded down
/// (18.16.6's own example: 600 Mbit/s is 1.667 ns, 6.667 quarters, 6).
constexpr uint8_t dsi_uix4(uint32_t bit_rate_hz) {
    if (bit_rate_hz == 0u) {
        return 0u;
    }
    const uint32_t q = static_cast<uint32_t>(4'000'000'000ull / bit_rate_hz);
    return q > 63u ? 63u : static_cast<uint8_t>(q);
}

/// CCR.TXECKDIV: the smallest divisor of the lane byte clock that keeps
/// the TX escape clock at or under 20 MHz; 0 and 1 stop the clock
/// (18.15.3), so never less than 2.
constexpr uint8_t dsi_escape_divider(uint32_t lane_byte_hz) {
    uint32_t d = 2;
    while (d < 255u && lane_byte_hz / d > dsi_escape_clock_max_hz) {
        ++d;
    }
    return static_cast<uint8_t>(d);
}

/// Nanoseconds as lane byte clock cycles, rounded up.
constexpr uint32_t dsi_cycles_for_ns(uint32_t lane_byte_hz, uint32_t ns) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ns) * lane_byte_hz + 999'999'999u) /
                                 1'000'000'000u);
}

enum class DsiLanes : uint8_t { one = 0, two = 1 };

/**
 * The lane transition times the host plans with (DSI_CLTCR, DSI_DLTCR,
 * DSI_PCONFR.SW_TIME), in lane byte clock cycles: the maximum time a
 * lane takes from low power to high speed and back, the maximum time a
 * read may take, and the wait after a Stop state before a high-speed
 * request.
 */
struct DsiPhyTiming {
    uint16_t clock_hs2lp = 0;    ///< 10 bits
    uint16_t clock_lp2hs = 0;    ///< 10 bits
    uint8_t data_hs2lp = 0;
    uint8_t data_lp2hs = 0;
    uint16_t max_read_time = 0;  ///< MRD_TIME, 15 bits
    uint8_t stop_wait = 0;       ///< SW_TIME
};

constexpr bool dsi_phy_timing_ok(const DsiPhyTiming& t) {
    return t.clock_hs2lp < 1024u && t.clock_lp2hs < 1024u && t.max_read_time < 32768u;
}

/**
 * The transition times from DS11189 table 46's maxima, with a margin of
 * half again and rounded up to lane byte clocks. Clock lane, LP to HS:
 * TLPX + TCLK-PREPARE + TCLK-ZERO + TCLK-PRE = 50 + 300 + 8 UI ns; HS to
 * LP: TCLK-POST + TCLK-TRAIL + THS-EXIT = (62 + 52 UI) + 60 + 100 ns.
 * Data lane, LP to HS: TLPX + THS-PREPARE + THS-ZERO = 50 + 145 + 10 UI
 * ns; HS to LP: THS-TRAIL + THS-EXIT = (60 + 8 UI) + 100 ns. The read
 * time is left at the field's maximum (a read is bounded by the verbs'
 * own waits) and the stop wait at ten cycles, several times TLPX.
 */
constexpr DsiPhyTiming dsi_phy_timing_for(uint32_t bit_rate_hz) {
    DsiPhyTiming t{};
    if (bit_rate_hz == 0u) {
        return t;
    }
    const uint32_t lane_byte_hz = dsi_lane_byte_hz(bit_rate_hz);
    // UI in picoseconds, the sums in nanoseconds with the UI terms
    // rounded up.
    const uint32_t ui_ps = static_cast<uint32_t>(1'000'000'000'000ull / bit_rate_hz);
    const auto ui_ns = [ui_ps](uint32_t n) { return (n * ui_ps + 999u) / 1000u; };
    const uint32_t clk_lp2hs_ns = 50u + 300u + ui_ns(8);
    const uint32_t clk_hs2lp_ns = 62u + ui_ns(52) + 60u + 100u;
    const uint32_t dat_lp2hs_ns = 50u + 145u + ui_ns(10);
    const uint32_t dat_hs2lp_ns = 60u + ui_ns(8) + 100u;
    const auto cycles = [lane_byte_hz](uint32_t ns) {
        return dsi_cycles_for_ns(lane_byte_hz, ns + ns / 2u);
    };
    t.clock_lp2hs = static_cast<uint16_t>(cycles(clk_lp2hs_ns));
    t.clock_hs2lp = static_cast<uint16_t>(cycles(clk_hs2lp_ns));
    t.data_lp2hs = static_cast<uint8_t>(cycles(dat_lp2hs_ns));
    t.data_hs2lp = static_cast<uint8_t>(cycles(dat_hs2lp_ns));
    t.max_read_time = 0x7FFFu;
    t.stop_wait = 10;
    return t;
}

/// The D-PHY as the host sees it (18.14.2): the lane count, the clock
/// lane in high speed or low power, the automatic clock lane control,
/// and the transition times.
struct DsiPhyConfig {
    DsiLanes lanes = DsiLanes::two;
    bool clock_lane_hs = true;          ///< CLCR.DPCC: the clock lane runs in high speed
    bool automatic_clock_lane = false;  ///< CLCR.ACR: the clock stopped between transmissions
    DsiPhyTiming timing{};
};

/**
 * The host's own protocol settings (18.14.3, 18.14.4): the two derived
 * clocks, the flow control bits, whether commands go out in low power
 * (every type at once - one link, one choice), the acknowledge
 * requests, the virtual channel, the timeout counters (0 = none, the
 * reset value) and the largest low-power packet a blanking period
 * carries.
 */
struct DsiHostConfig {
    uint8_t escape_divider = 4;         ///< CCR.TXECKDIV, from dsi_escape_divider()
    uint8_t timeout_divider = 1;        ///< CCR.TOCKDIV
    bool eotp_transmit = false;         ///< PCR.ETTXE
    bool eotp_receive = false;          ///< PCR.ETRXE
    bool bus_turn_around = true;        ///< PCR.BTAE - a read needs it
    bool ecc_receive = true;            ///< PCR.ECCRXE
    bool crc_receive = true;            ///< PCR.CRCRXE
    bool commands_low_power = true;     ///< CMCR's eleven short-packet and read-size bits
    bool long_writes_low_power = true;  ///< CMCR's GLWTX and DLWTX - the LTDC's memory writes are DCS long writes
    bool acknowledge_request = false;   ///< CMCR.ARE
    bool te_acknowledge = false;        ///< CMCR.TEARE
    uint8_t virtual_channel = 0;        ///< GVCIDR and LVCIDR, 0..3
    uint16_t hs_tx_timeout = 0;         ///< TCCR0.HSTX_TOCNT
    uint16_t lp_rx_timeout = 0;         ///< TCCR0.LPRX_TOCNT
    uint16_t hs_read_timeout = 0;       ///< TCCR1
    uint16_t lp_read_timeout = 0;       ///< TCCR2
    uint16_t hs_write_timeout = 0;      ///< TCCR3.HSWR_TOCNT
    bool presp_once_per_frame = false;  ///< TCCR3.PM
    uint16_t lp_write_timeout = 0;      ///< TCCR4
    uint16_t bta_timeout = 0;           ///< TCCR5
    uint8_t lp_largest_packet = 16;     ///< LPMCR.LPSIZE, bytes in VSA/VBP/VFP
    uint8_t lp_largest_packet_active = 0;  ///< LPMCR.VLPSIZE, bytes in VACT
};

constexpr bool dsi_host_config_ok(const DsiHostConfig& c) {
    return c.escape_divider >= 2u && c.virtual_channel < 4u;
}

/// The colour coding of the LTDC interface (LCOLCR.COLC and
/// WCFGR.COLMUX, one code for both): which bits of the LTDC's 24 the
/// host packs, and into which pixel packet type.
enum class DsiColor : uint8_t {
    rgb565_1 = 0,
    rgb565_2 = 1,
    rgb565_3 = 2,
    rgb666_1 = 3,
    rgb666_2 = 4,
    rgb888 = 5,
};

/// The video mode type (VMCR.VMT): the sync pulses or only their events
/// reproduced on the link, or the line's pixels sent as one burst with
/// the rest of the line in low power.
enum class DsiVideoType : uint8_t { sync_pulses = 0, sync_events = 1, burst = 2 };

/**
 * Video mode (18.14.6): the type, which periods may return to low power
 * when time allows, whether commands go in low power, the acknowledge
 * at the end of a frame, and the packet arithmetic - the pixels in one
 * packet (0: the whole line), the chunks and the null packet a
 * non-burst line is cut into.
 */
struct DsiVideoConfig {
    DsiVideoType type = DsiVideoType::sync_pulses;
    bool lp_hfp = true;
    bool lp_hbp = true;
    bool lp_vact = true;
    bool lp_vfp = true;
    bool lp_vbp = true;
    bool lp_vsa = true;
    bool lp_commands = true;      ///< VMCR.LPCE
    bool frame_bta_ack = false;   ///< VMCR.FBTAAE
    bool loosely_packed = false;  ///< LCOLCR.LPE, the 18-bit variant
    uint16_t packet_pixels = 0;   ///< VPSIZE, 14 bits; 0 takes the line's width
    uint16_t chunks = 0;          ///< NUMC, 13 bits
    uint16_t null_bytes = 0;      ///< NPSIZE, 13 bits
};

/// The host's frame in its own units (18.14.6): the horizontal periods in
/// lane byte clocks, the vertical ones in lines, the width in pixels.
struct DsiVideoTiming {
    uint16_t hsa = 0;     ///< 12 bits
    uint16_t hbp = 0;     ///< 12 bits
    uint16_t hline = 0;   ///< 15 bits: HSA + HBP + HACT + HFP
    uint16_t vsa = 0;     ///< 10 bits
    uint16_t vbp = 0;     ///< 10 bits
    uint16_t vfp = 0;     ///< 10 bits
    uint16_t va = 0;      ///< 14 bits
    uint16_t width = 0;   ///< the active pixels of a line
};

constexpr bool dsi_video_timing_ok(const DsiVideoTiming& t) {
    return t.hsa < 4096u && t.hbp < 4096u && t.hline < 32768u && t.vsa < 1024u && t.vbp < 1024u &&
           t.vfp < 1024u && t.va < 16384u && t.width != 0u && t.width < 16384u &&
           t.hline > t.hsa + t.hbp;
}

/// A pixel count as lane byte clocks at the two clocks' ratio, rounded
/// to nearest - item 3.
constexpr uint32_t dsi_lane_cycles(uint32_t pixels, uint32_t pixel_hz, uint32_t lane_byte_hz) {
    if (pixel_hz == 0u) {
        return 0u;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(pixels) * lane_byte_hz + pixel_hz / 2u) /
                                 pixel_hz);
}

/// The LTDC's timing converted for the host (18.14.6).
constexpr DsiVideoTiming dsi_video_timing(const LtdcTiming& t, uint32_t pixel_hz, uint32_t lane_byte_hz) {
    DsiVideoTiming v{};
    v.hsa = static_cast<uint16_t>(dsi_lane_cycles(t.hsync, pixel_hz, lane_byte_hz));
    v.hbp = static_cast<uint16_t>(dsi_lane_cycles(t.hbp, pixel_hz, lane_byte_hz));
    v.hline = static_cast<uint16_t>(dsi_lane_cycles(ltdc_total_width(t), pixel_hz, lane_byte_hz));
    v.vsa = t.vsync;
    v.vbp = t.vbp;
    v.vfp = t.vfp;
    v.va = t.height;
    v.width = t.width;
    return v;
}

/// The bytes one line's pixels take on the link for a colour coding
/// (3, 2 or 2.25 a pixel), plus the packet's header and checksum: what
/// a line's lane byte clocks must exceed, divided by the lanes, for a
/// non-burst line to fit.
constexpr uint32_t dsi_line_payload_bytes(uint16_t width, DsiColor c) {
    switch (c) {
        case DsiColor::rgb888: return 3u * width + 6u;
        case DsiColor::rgb666_1:
        case DsiColor::rgb666_2: return (9u * width + 3u) / 4u + 6u;
        default: return 2u * width + 6u;
    }
}

/// Whether a line fits its lane byte clock budget on `lanes`.
constexpr bool dsi_line_fits(const DsiVideoTiming& v, DsiColor c, DsiLanes lanes) {
    const uint32_t lane_count = lanes == DsiLanes::two ? 2u : 1u;
    const uint32_t needed = (dsi_line_payload_bytes(v.width, c) + lane_count - 1u) / lane_count;
    return static_cast<uint32_t>(v.hline) > needed + v.hsa + v.hbp;
}

/// The video mode pattern generator (18.11): eight colour bars or the
/// BER pattern, with no pixel from the LTDC.
enum class DsiPattern : uint8_t { off, vertical_bars, horizontal_bars, ber };

/// Everything `init()` writes, as one value: the PLL, the D-PHY, the
/// host - the bit rate stated beside the PLL so the timings derive from
/// it.
struct DsiConfig {
    DsiPllConfig pll{};
    uint32_t bit_rate_hz = 0;
    DsiPhyConfig phy{};
    DsiHostConfig host{};
};

constexpr bool dsi_config_ok(uint32_t clkin_hz, const DsiConfig& c) {
    return dsi_pll_config_ok(clkin_hz, c.pll) && dsi_pll_bit_rate_hz(clkin_hz, c.pll) == c.bit_rate_hz &&
           dsi_phy_timing_ok(c.phy.timing) && dsi_host_config_ok(c.host);
}

/// A configuration for a lane bit rate off HSE: the PLL solved, the
/// timings derived, the host at its defaults. `pll.ndiv == 0` when the
/// rate is not reachable.
constexpr DsiConfig dsi_config_for(uint32_t clkin_hz, uint32_t bit_rate_hz) {
    DsiConfig c{};
    c.pll = dsi_pll_for(clkin_hz, bit_rate_hz);
    c.bit_rate_hz = bit_rate_hz;
    c.phy.timing = dsi_phy_timing_for(bit_rate_hz);
    c.host.escape_divider = dsi_escape_divider(dsi_lane_byte_hz(bit_rate_hz));
    return c;
}

/// What a verb of the generic interface reports.
enum class DsiStatus : uint8_t {
    ok,
    refused,   ///< the arguments break a rule above, or the host is not enabled; nothing sent
    timeout,   ///< the bounded wait on a FIFO flag ran out
    error,     ///< the host raised an error register bit during the exchange
};

/// The wrapper's five events (table 134) and the two error registers
/// (18.15.41, 18.15.42), as `isr()` reports them.
struct DsiEvents {
    bool tearing_effect = false;
    bool end_of_refresh = false;
    bool pll_locked = false;
    bool pll_unlocked = false;
    bool regulator_ready = false;
    uint32_t errors0 = 0;   ///< DSI_ISR0's bits: AE0..15, PE0..4
    uint32_t errors1 = 0;   ///< DSI_ISR1's bits: TOHSTX .. GPRXE
};

/// The D-PHY's status lines (DSI_PSR).
struct DsiLaneStatus {
    bool clock_stop = false;        ///< PSSC
    bool clock_ulps = false;        ///< UANC low: ULPS active (the bit is active-not)
    bool data0_stop = false;        ///< PSS0
    bool data0_ulps = false;        ///< UAN0 low
    bool data0_rx_ulps = false;     ///< RUE0
    bool data1_stop = false;        ///< PSS1
    bool data1_ulps = false;        ///< UAN1 low
    bool direction_rx = false;      ///< PD
};

#if defined(DSI_BASE)

// =============================================================================
// The block
// =============================================================================

struct Dsi {
    Dsi() = delete;

    static DSI_TypeDef& regs() { return *DSI; }

    static constexpr IRQn_Type irq = dsi_irq();

    /// A bounded wait's default: a regulator's or a PLL's start is
    /// microseconds (tLOCK 200 us at most), a command's FIFO wait the
    /// same order.
    static constexpr uint32_t default_spins = 2'000'000u;

    // ---- the clock gate and the reset line (APB2) ------------------------------

    static void clock(bool on) { Rcc::apb2_clock(dsi_clock_mask(), on); }
    static bool clock() { return Rcc::apb2_clock(dsi_clock_mask()); }
    /// RCC_APB2RSTR.DSIRST: the wrapper and the host back to their reset
    /// values, the regulator and the PLL off.
    static void reset_block() { Rcc::apb2_reset(dsi_reset_mask()); }

    /// DSI_VR, the host's version word.
    static uint32_t version() { return regs().VR; }

    /// RCC_DCKCFGR.DSISEL: whether the lane byte clock comes from the
    /// main PLL's R output instead of the D-PHY. Read only here: the clock
    /// task fixes PLLR at 2, whose output is above the 62.5 MHz ceiling at
    /// every rate this stratum runs, so nothing selects it.
    static bool byte_clock_from_pllr() {
        return (RCC->DCKCFGR & dsi_byte_clock_select_mask()) != 0u;
    }

    // ---- the regulator (18.12.6) ----------------------------------------------

    /// WRPCR.REGEN, waited for through WISR.RRS; the D-PHY has no power
    /// bit of its own and follows the regulator. False when the wait runs
    /// out (the bit left set: the regulator may still come).
    static bool regulator(bool on, uint32_t spins = default_spins) {
        if (on) {
            regs().WRPCR |= DSI_WRPCR_REGEN;
            return wait(regs().WISR, DSI_WISR_RRS, true, spins);
        }
        regs().WRPCR &= ~DSI_WRPCR_REGEN;
        return true;
    }
    static bool regulator_ready() { return (regs().WISR & DSI_WISR_RRS) != 0u; }

    // ---- the PLL (18.12.5) ----------------------------------------------------

    /// The PLL stopped, the triple written, the PLL started and waited
    /// for (WISR.PLLLS). Refused - and the PLL left off - when the triple
    /// breaks the ranges at this CLKIN; false when the lock does not come.
    static bool pll(uint32_t clkin_hz, const DsiPllConfig& c, uint32_t spins = default_spins) {
        if (!dsi_pll_config_ok(clkin_hz, c)) {
            return false;
        }
        regs().WRPCR &= ~DSI_WRPCR_PLLEN;
        regs().WRPCR = (regs().WRPCR & ~(DSI_WRPCR_PLL_NDIV_Msk | DSI_WRPCR_PLL_IDF_Msk | DSI_WRPCR_PLL_ODF_Msk)) |
                       (static_cast<uint32_t>(c.ndiv) << DSI_WRPCR_PLL_NDIV_Pos) |
                       (static_cast<uint32_t>(c.idf) << DSI_WRPCR_PLL_IDF_Pos) |
                       (static_cast<uint32_t>(odf_code(c.odf)) << DSI_WRPCR_PLL_ODF_Pos);
        regs().WRPCR |= DSI_WRPCR_PLLEN;
        return wait(regs().WISR, DSI_WISR_PLLLS, true, spins);
    }
    static void pll_off() { regs().WRPCR &= ~DSI_WRPCR_PLLEN; }
    static bool pll_on() { return (regs().WRPCR & DSI_WRPCR_PLLEN) != 0u; }
    static bool pll_locked() { return (regs().WISR & DSI_WISR_PLLLS) != 0u; }
    /// The triple as the register holds it.
    static DsiPllConfig pll_config() {
        const uint32_t v = regs().WRPCR;
        DsiPllConfig c{};
        c.ndiv = static_cast<uint8_t>((v & DSI_WRPCR_PLL_NDIV_Msk) >> DSI_WRPCR_PLL_NDIV_Pos);
        c.idf = static_cast<uint8_t>((v & DSI_WRPCR_PLL_IDF_Msk) >> DSI_WRPCR_PLL_IDF_Pos);
        c.odf = static_cast<uint8_t>(1u << ((v & DSI_WRPCR_PLL_ODF_Msk) >> DSI_WRPCR_PLL_ODF_Pos));
        return c;
    }

    // ---- the D-PHY (18.14.2) --------------------------------------------------

    /// WPCR0.UIX4 from the bit rate, PCONFR (the lanes, the stop wait),
    /// CLCR (the clock lane's mode), CLTCR and DLTCR (the transition
    /// times - both clock-lane fields at the larger of the two, ES0321
    /// 2.8.2). Refused while the host is enabled or the timing breaks a
    /// field.
    static bool phy(const DsiPhyConfig& c, uint32_t bit_rate_hz) {
        if (enabled() || !dsi_phy_timing_ok(c.timing) || bit_rate_hz == 0u) {
            return false;
        }
        regs().WPCR[0] = (regs().WPCR[0] & ~DSI_WPCR0_UIX4_Msk) |
                         (static_cast<uint32_t>(dsi_uix4(bit_rate_hz)) << DSI_WPCR0_UIX4_Pos);
        regs().PCONFR = (static_cast<uint32_t>(c.lanes) << DSI_PCONFR_NL_Pos) |
                        (static_cast<uint32_t>(c.timing.stop_wait) << DSI_PCONFR_SW_TIME_Pos);
        regs().CLCR = (c.clock_lane_hs ? DSI_CLCR_DPCC : 0u) | (c.automatic_clock_lane ? DSI_CLCR_ACR : 0u);
        const uint32_t clock_time =
            c.timing.clock_hs2lp > c.timing.clock_lp2hs ? c.timing.clock_hs2lp : c.timing.clock_lp2hs;
        regs().CLTCR = (clock_time << DSI_CLTCR_HS2LP_TIME_Pos) | (clock_time << DSI_CLTCR_LP2HS_TIME_Pos);
        regs().DLTCR = (static_cast<uint32_t>(c.timing.data_hs2lp) << DSI_DLTCR_HS2LP_TIME_Pos) |
                       (static_cast<uint32_t>(c.timing.data_lp2hs) << DSI_DLTCR_LP2HS_TIME_Pos) |
                       (static_cast<uint32_t>(c.timing.max_read_time) << DSI_DLTCR_MRD_TIME_Pos);
        return true;
    }

    /// CLCR.DPCC alone, live: the clock lane into high speed or back to
    /// low power with everything else in place - a panel's receiver locks
    /// onto the clock lane's LP-to-HS entry, so a panel reset under a
    /// clock lane already in high speed takes no high-speed packet until
    /// it sees one (measured; item 11 in the file's head).
    static void clock_lane(bool high_speed) { bit(regs().CLCR, DSI_CLCR_DPCC, high_speed); }
    static bool clock_lane() { return (regs().CLCR & DSI_CLCR_DPCC) != 0u; }

    static uint8_t uix4() {
        return static_cast<uint8_t>((regs().WPCR[0] & DSI_WPCR0_UIX4_Msk) >> DSI_WPCR0_UIX4_Pos);
    }
    static DsiLanes lanes() {
        return static_cast<DsiLanes>((regs().PCONFR & DSI_PCONFR_NL_Msk) >> DSI_PCONFR_NL_Pos);
    }
    static DsiPhyTiming phy_timing() {
        DsiPhyTiming t{};
        const uint32_t cl = regs().CLTCR;
        const uint32_t dl = regs().DLTCR;
        t.clock_hs2lp = static_cast<uint16_t>((cl & DSI_CLTCR_HS2LP_TIME_Msk) >> DSI_CLTCR_HS2LP_TIME_Pos);
        t.clock_lp2hs = static_cast<uint16_t>((cl & DSI_CLTCR_LP2HS_TIME_Msk) >> DSI_CLTCR_LP2HS_TIME_Pos);
        t.data_hs2lp = static_cast<uint8_t>((dl & DSI_DLTCR_HS2LP_TIME_Msk) >> DSI_DLTCR_HS2LP_TIME_Pos);
        t.data_lp2hs = static_cast<uint8_t>((dl & DSI_DLTCR_LP2HS_TIME_Msk) >> DSI_DLTCR_LP2HS_TIME_Pos);
        t.max_read_time = static_cast<uint16_t>((dl & DSI_DLTCR_MRD_TIME_Msk) >> DSI_DLTCR_MRD_TIME_Pos);
        t.stop_wait = static_cast<uint8_t>((regs().PCONFR & DSI_PCONFR_SW_TIME_Msk) >> DSI_PCONFR_SW_TIME_Pos);
        return t;
    }

    /// DSI_PSR, decoded.
    static DsiLaneStatus lane_status() {
        const uint32_t v = regs().PSR;
        DsiLaneStatus s{};
        s.clock_stop = (v & DSI_PSR_PSSC) != 0u;
        s.clock_ulps = (v & DSI_PSR_UANC) == 0u;
        s.data0_stop = (v & DSI_PSR_PSS0) != 0u;
        s.data0_ulps = (v & DSI_PSR_UAN0) == 0u;
        s.data0_rx_ulps = (v & DSI_PSR_RUE0) != 0u;
        s.data1_stop = (v & DSI_PSR_PSS1) != 0u;
        s.data1_ulps = (v & DSI_PSR_UAN1) == 0u;
        s.direction_rx = (v & DSI_PSR_PD) != 0u;
        return s;
    }

    /// DSI_PUCR: the ultra-low-power state requested or left on the clock
    /// lane and on the data lanes; the request bits are levels, so a
    /// request is cleared by its exit and the reverse.
    static void ulps(bool clock_lane, bool data_lanes, bool enter) {
        uint32_t v = 0;
        if (clock_lane) {
            v |= enter ? DSI_PUCR_URCL : DSI_PUCR_UECL;
        }
        if (data_lanes) {
            v |= enter ? DSI_PUCR_URDL : DSI_PUCR_UEDL;
        }
        regs().PUCR = v;
    }

    // ---- the host's timing and flow control (18.14.3, 18.14.4) -----------------

    /// CCR, PCR, GVCIDR, LVCIDR, CMCR, the six timeout registers and
    /// LPMCR. Refused while enabled or outside dsi_host_config_ok().
    static bool host(const DsiHostConfig& c) {
        if (enabled() || !dsi_host_config_ok(c)) {
            return false;
        }
        regs().CCR = (static_cast<uint32_t>(c.timeout_divider) << DSI_CCR_TOCKDIV_Pos) |
                     (static_cast<uint32_t>(c.escape_divider) << DSI_CCR_TXECKDIV_Pos);
        regs().PCR = (c.eotp_transmit ? DSI_PCR_ETTXE : 0u) | (c.eotp_receive ? DSI_PCR_ETRXE : 0u) |
                     (c.bus_turn_around ? DSI_PCR_BTAE : 0u) | (c.ecc_receive ? DSI_PCR_ECCRXE : 0u) |
                     (c.crc_receive ? DSI_PCR_CRCRXE : 0u);
        regs().GVCIDR = c.virtual_channel;
        regs().LVCIDR = c.virtual_channel;
        regs().CMCR = (c.commands_low_power ? short_type_bits : 0u) | (c.long_writes_low_power ? long_type_bits : 0u) |
                      (c.acknowledge_request ? DSI_CMCR_ARE : 0u) | (c.te_acknowledge ? DSI_CMCR_TEARE : 0u);
        regs().TCCR[0] = (static_cast<uint32_t>(c.hs_tx_timeout) << DSI_TCCR0_HSTX_TOCNT_Pos) |
                         (static_cast<uint32_t>(c.lp_rx_timeout) << DSI_TCCR0_LPRX_TOCNT_Pos);
        regs().TCCR[1] = c.hs_read_timeout;
        regs().TCCR[2] = c.lp_read_timeout;
        regs().TCCR[3] = (c.presp_once_per_frame ? DSI_TCCR3_PM : 0u) | c.hs_write_timeout;
        regs().TCCR[4] = c.lp_write_timeout;
        regs().TCCR[5] = c.bta_timeout;
        regs().LPMCR = (static_cast<uint32_t>(c.lp_largest_packet) << DSI_LPMCR_LPSIZE_Pos) |
                       (static_cast<uint32_t>(c.lp_largest_packet_active) << DSI_LPMCR_VLPSIZE_Pos);
        return true;
    }

    static uint8_t escape_divider() {
        return static_cast<uint8_t>((regs().CCR & DSI_CCR_TXECKDIV_Msk) >> DSI_CCR_TXECKDIV_Pos);
    }
    static bool commands_low_power() { return (regs().CMCR & short_type_bits) == short_type_bits; }
    static bool long_writes_low_power() { return (regs().CMCR & long_type_bits) == long_type_bits; }
    static uint8_t virtual_channel() { return static_cast<uint8_t>(regs().GVCIDR & DSI_GVCIDR_VCID_Msk); }

    // ---- the LTDC interface (18.14.5) -------------------------------------------

    /// LCOLCR.COLC and WCFGR.COLMUX (one code, two registers), LCOLCR.LPE,
    /// and LPCR's three polarities from the LTDC's own timing - active low
    /// in the LTDC's words is the bit set here. Refused while enabled.
    static bool colour(DsiColor c, const LtdcTiming& t, bool loosely_packed = false) {
        if (enabled()) {
            return false;
        }
        regs().LCOLCR = (static_cast<uint32_t>(c) << DSI_LCOLCR_COLC_Pos) | (loosely_packed ? DSI_LCOLCR_LPE : 0u);
        regs().WCFGR = (regs().WCFGR & ~DSI_WCFGR_COLMUX_Msk) | (static_cast<uint32_t>(c) << DSI_WCFGR_COLMUX_Pos);
        regs().LPCR = (t.hsync_polarity == LtdcPolarity::active_low ? DSI_LPCR_HSP : 0u) |
                      (t.vsync_polarity == LtdcPolarity::active_low ? DSI_LPCR_VSP : 0u) |
                      (t.de_polarity == LtdcPolarity::active_low ? DSI_LPCR_DEP : 0u);
        return true;
    }

    static DsiColor colour() {
        return static_cast<DsiColor>((regs().LCOLCR & DSI_LCOLCR_COLC_Msk) >> DSI_LCOLCR_COLC_Pos);
    }

    /// WCFGR.TESRC/TEPOL: the tearing effect from the dedicated pin
    /// (ES0321 2.8.1's workaround) on the edge given, or from the link.
    /// Refused while enabled.
    static bool te_source(bool from_pin, bool falling_edge = false) {
        if (enabled()) {
            return false;
        }
        regs().WCFGR = (regs().WCFGR & ~(DSI_WCFGR_TESRC | DSI_WCFGR_TEPOL)) |
                       (from_pin ? DSI_WCFGR_TESRC : 0u) | (falling_edge ? DSI_WCFGR_TEPOL : 0u);
        return true;
    }

    /// WCFGR.AR and VSPOL, the adapted command mode's automatic refresh
    /// and the VSync edge the LTDC is halted on. Refused while enabled.
    static bool adapted_refresh(bool automatic, bool halt_on_rising) {
        if (enabled()) {
            return false;
        }
        regs().WCFGR = (regs().WCFGR & ~(DSI_WCFGR_AR | DSI_WCFGR_VSPOL)) | (automatic ? DSI_WCFGR_AR : 0u) |
                       (halt_on_rising ? DSI_WCFGR_VSPOL : 0u);
        return true;
    }

    // ---- the two modes (18.14.6, 18.14.7) -----------------------------------------

    /// Command mode: MCR.CMDM and WCFGR.DSIM set, and the size of an LTDC
    /// memory-write command in pixels (LCCR.CMDSIZE) for the adapted
    /// command mode. Refused while enabled.
    static bool command_mode(uint16_t command_pixels) {
        if (enabled()) {
            return false;
        }
        regs().MCR = DSI_MCR_CMDM;
        regs().WCFGR |= DSI_WCFGR_DSIM;
        regs().LCCR = command_pixels;
        return true;
    }

    /// Video mode: MCR.CMDM and WCFGR.DSIM clear, VMCR from the config,
    /// the packet arithmetic and the eight timing registers. Refused
    /// while enabled, outside the fields, or when a line's pixels do not
    /// fit its lane byte clocks on the lanes configured.
    static bool video(const DsiVideoConfig& c, const DsiVideoTiming& t) {
        if (enabled() || !dsi_video_timing_ok(t) || c.chunks >= 8192u || c.null_bytes >= 8192u ||
            c.packet_pixels >= 16384u || !dsi_line_fits(t, colour(), lanes())) {
            return false;
        }
        regs().MCR = 0u;
        regs().WCFGR &= ~DSI_WCFGR_DSIM;
        const uint32_t pge = regs().VMCR & (DSI_VMCR_PGE | DSI_VMCR_PGM | DSI_VMCR_PGO);
        regs().VMCR = pge | (static_cast<uint32_t>(c.type) << DSI_VMCR_VMT_Pos) |
                      (c.lp_vsa ? DSI_VMCR_LPVSAE : 0u) | (c.lp_vbp ? DSI_VMCR_LPVBPE : 0u) |
                      (c.lp_vfp ? DSI_VMCR_LPVFPE : 0u) | (c.lp_vact ? DSI_VMCR_LPVAE : 0u) |
                      (c.lp_hbp ? DSI_VMCR_LPHBPE : 0u) | (c.lp_hfp ? DSI_VMCR_LPHFPE : 0u) |
                      (c.frame_bta_ack ? DSI_VMCR_FBTAAE : 0u) | (c.lp_commands ? DSI_VMCR_LPCE : 0u);
        regs().LCOLCR = (regs().LCOLCR & ~DSI_LCOLCR_LPE) | (c.loosely_packed ? DSI_LCOLCR_LPE : 0u);
        regs().VPCR = c.packet_pixels != 0u ? c.packet_pixels : t.width;
        regs().VCCR = c.chunks;
        regs().VNPCR = c.null_bytes;
        regs().VHSACR = t.hsa;
        regs().VHBPCR = t.hbp;
        regs().VLCR = t.hline;
        regs().VVSACR = t.vsa;
        regs().VVBPCR = t.vbp;
        regs().VVFPCR = t.vfp;
        regs().VVACR = t.va;
        return true;
    }

    static bool in_command_mode() { return (regs().MCR & DSI_MCR_CMDM) != 0u; }

    /// The frame as the registers hold it.
    static DsiVideoTiming video_timing() {
        DsiVideoTiming t{};
        t.hsa = static_cast<uint16_t>(regs().VHSACR & DSI_VHSACR_HSA_Msk);
        t.hbp = static_cast<uint16_t>(regs().VHBPCR & DSI_VHBPCR_HBP_Msk);
        t.hline = static_cast<uint16_t>(regs().VLCR & DSI_VLCR_HLINE_Msk);
        t.vsa = static_cast<uint16_t>(regs().VVSACR & DSI_VVSACR_VSA_Msk);
        t.vbp = static_cast<uint16_t>(regs().VVBPCR & DSI_VVBPCR_VBP_Msk);
        t.vfp = static_cast<uint16_t>(regs().VVFPCR & DSI_VVFPCR_VFP_Msk);
        t.va = static_cast<uint16_t>(regs().VVACR & DSI_VVACR_VA_Msk);
        t.width = static_cast<uint16_t>(regs().VPCR & DSI_VPCR_VPSIZE_Msk);
        return t;
    }

    /// The pattern generator (18.14.8): VMCR's PGE/PGM/PGO, written live -
    /// the video registers it uses are whatever `video()` left.
    static void pattern(DsiPattern p) {
        uint32_t v = regs().VMCR & ~(DSI_VMCR_PGE | DSI_VMCR_PGM | DSI_VMCR_PGO);
        switch (p) {
            case DsiPattern::vertical_bars: v |= DSI_VMCR_PGE; break;
            case DsiPattern::horizontal_bars: v |= DSI_VMCR_PGE | DSI_VMCR_PGO; break;
            case DsiPattern::ber: v |= DSI_VMCR_PGE | DSI_VMCR_PGM; break;
            default: break;
        }
        regs().VMCR = v;
    }
    static bool pattern_on() { return (regs().VMCR & DSI_VMCR_PGE) != 0u; }

    /// The video shadow registers (18.15.47): the "current" copies take
    /// the configuration on UR, and ES0321 2.8.3 makes the immediate
    /// update a sequence - 0 then EN + UR, checked by UR auto-clearing,
    /// repeated while it has not. False when the retries ran out.
    static bool shadow_update(uint32_t retries = 8) {
        for (uint32_t i = 0; i < retries; ++i) {
            regs().VSCR = 0u;
            regs().VSCR = DSI_VSCR_EN | DSI_VSCR_UR;
            for (uint32_t spins = 100'000u; spins != 0u; --spins) {
                if ((regs().VSCR & DSI_VSCR_UR) == 0u) {
                    return true;
                }
            }
        }
        return false;
    }
    static bool shadowed() { return (regs().VSCR & DSI_VSCR_EN) != 0u; }
    static void shadow(bool on) { regs().VSCR = on ? DSI_VSCR_EN : 0u; }

    // ---- bring-up -------------------------------------------------------------

    /// The programming procedure's first stretch (18.14.1 steps 5 .. 9):
    /// the gate opened, the host disabled, the regulator started and
    /// waited for, the PLL configured and locked, the D-PHY and the host
    /// timings written. The LTDC interface, the mode and the enable are
    /// the caller's next three verbs, because they need the panel's own
    /// numbers. False and nothing further when a step fails; a
    /// configuration outside the rules is refused at compile time.
    template <DsiConfig cfg, uint32_t clkin_hz>
    static bool init() {
        static_assert(dsi_config_ok(clkin_hz, cfg),
                      "brio Dsi: the PLL triple must be exact and inside RM0386 18.12.5's and "
                      "DS11189 table 47's ranges (CLKIN 4..100 MHz, the PFD input 8..25 MHz, the "
                      "VCO 500..1000 MHz, the output 31.25..500 MHz; NDIV 10..125, IDF 1..7, "
                      "ODF 1/2/4/8), the escape divider at least 2, the virtual channel 0..3");
        return init(clkin_hz, cfg);
    }

    static bool init(uint32_t clkin_hz, const DsiConfig& cfg) {
        if (!dsi_config_ok(clkin_hz, cfg)) {
            return false;
        }
        clock(true);
        disable();
        if (!regulator(true)) {
            return false;
        }
        if (!pll(clkin_hz, cfg.pll)) {
            return false;
        }
        return phy(cfg.phy, cfg.bit_rate_hz) && host(cfg.host);
    }

    /// Steps 12 .. 15 of 18.14.1: the D-PHY's digital section and its
    /// clock lane module enabled, then the host, then the wrapper.
    static void enable() {
        regs().PCTLR = DSI_PCTLR_DEN;
        regs().PCTLR = DSI_PCTLR_DEN | DSI_PCTLR_CKE;
        regs().CR = DSI_CR_EN;
        regs().WCR |= DSI_WCR_DSIEN;
    }

    /// The wrapper and the host disabled - the host "under reset"
    /// (18.15.2) - and the D-PHY's digital section with them.
    static void disable() {
        regs().WCR &= ~(DSI_WCR_DSIEN | DSI_WCR_LTDCEN);
        regs().CR = 0u;
        regs().PCTLR = 0u;
    }

    /// Either half enabled: a configuration must not be written then.
    static bool enabled() {
        return (regs().CR & DSI_CR_EN) != 0u || (regs().WCR & DSI_WCR_DSIEN) != 0u;
    }

    /// Everything down: disabled, the PLL and the regulator off, the gate
    /// closed. The pads of the D-PHY are its own.
    static void release() {
        if (clock()) {
            disable();
            pll_off();
            (void)regulator(false);
            clock(false);
        }
    }

    // ---- the wrapper's controls (18.16.2) ------------------------------------

    /// WCR.LTDCEN - in video mode the stream starts with the LTDC itself
    /// (18.14.1 step 18); in adapted command mode this launches one frame.
    static void ltdc_flow(bool on) { bit(regs().WCR, DSI_WCR_LTDCEN, on); }
    static bool ltdc_flow() { return (regs().WCR & DSI_WCR_LTDCEN) != 0u; }
    /// WCR.SHTDN: the display shut down (the shutdown-peripheral packet
    /// goes out) or turned back on, in video mode.
    static void shutdown(bool off) { bit(regs().WCR, DSI_WCR_SHTDN, off); }
    static bool shutdown() { return (regs().WCR & DSI_WCR_SHTDN) != 0u; }
    /// WCR.COLM: the eight-colour mode packet, or full colour.
    static void eight_colours(bool on) { bit(regs().WCR, DSI_WCR_COLM, on); }
    static bool eight_colours() { return (regs().WCR & DSI_WCR_COLM) != 0u; }
    /// WISR.BUSY: a frame in flight in adapted command mode.
    static bool busy() { return (regs().WISR & DSI_WISR_BUSY) != 0u; }

    // ---- the generic interface (18.15.24 .. 18.15.26) --------------------------

    /// A short packet: the data type, the two data bytes in the header,
    /// nothing in the payload. Refused unless enabled.
    static DsiStatus short_write(DsiDataType dt, uint8_t p0, uint8_t p1, uint32_t spins = default_spins) {
        if (!enabled()) {
            return DsiStatus::refused;
        }
        if (!wait(regs().GPSR, DSI_GPSR_CMDFE, true, spins)) {
            return DsiStatus::timeout;
        }
        regs().GHCR = header(dt, p0, p1);
        return DsiStatus::ok;
    }

    /// A long packet: the payload into the write FIFO a word at a time,
    /// then the header with the byte count. Refused unless enabled or
    /// past a word count's 16 bits.
    static DsiStatus long_write(DsiDataType dt, const uint8_t* payload, uint16_t len, uint32_t spins = default_spins) {
        if (!enabled() || (len != 0u && payload == nullptr)) {
            return DsiStatus::refused;
        }
        if (!wait(regs().GPSR, DSI_GPSR_CMDFE, true, spins)) {
            return DsiStatus::timeout;
        }
        for (uint16_t i = 0; i < len; i += 4u) {
            if (!wait(regs().GPSR, DSI_GPSR_PWRFF, false, spins)) {
                return DsiStatus::timeout;
            }
            uint32_t w = 0;
            for (uint8_t b = 0; b < 4u && static_cast<uint16_t>(i + b) < len; ++b) {
                w |= static_cast<uint32_t>(payload[i + b]) << (8u * b);
            }
            regs().GPDR = w;
        }
        regs().GHCR = header(dt, static_cast<uint8_t>(len & 0xFFu), static_cast<uint8_t>(len >> 8));
        return DsiStatus::ok;
    }

    /// A read (item 6): the maximum return packet size set to `len`, the
    /// read request sent, the response waited for (RCB down and the read
    /// FIFO holding something) and drained from GPDR - `len` bytes into
    /// `buf`, the rest discarded. `error` when ISR1 raised a generic
    /// payload or reception error meanwhile; the two error registers are
    /// read (and so cleared) by this verb.
    static DsiStatus read(DsiDataType dt, uint8_t p0, uint8_t p1, uint8_t* buf, uint16_t len,
                          uint32_t spins = default_spins) {
        if (!enabled() || buf == nullptr || len == 0u) {
            return DsiStatus::refused;
        }
        (void)regs().ISR[1];
        // A response a program never collected (a read that timed out and
        // was answered later) would be handed to this one: drained first.
        while ((regs().GPSR & DSI_GPSR_PRDFE) == 0u) {
            (void)regs().GPDR;
        }
        DsiStatus st = short_write(DsiDataType::set_maximum_return_packet_size,
                                   static_cast<uint8_t>(len & 0xFFu), static_cast<uint8_t>(len >> 8), spins);
        if (st != DsiStatus::ok) {
            return st;
        }
        st = short_write(dt, p0, p1, spins);
        if (st != DsiStatus::ok) {
            return st;
        }
        bool arrived = false;
        for (uint32_t i = 0; i < spins; ++i) {
            const uint32_t s = regs().GPSR;
            if ((s & DSI_GPSR_RCB) == 0u && (s & DSI_GPSR_PRDFE) == 0u) {
                arrived = true;
                break;
            }
        }
        if (!arrived) {
            return DsiStatus::timeout;
        }
        uint16_t got = 0;
        while ((regs().GPSR & DSI_GPSR_PRDFE) == 0u) {
            const uint32_t w = regs().GPDR;
            for (uint8_t b = 0; b < 4u; ++b) {
                if (got < len) {
                    buf[got] = static_cast<uint8_t>(w >> (8u * b));
                }
                ++got;
            }
        }
        const uint32_t e1 = regs().ISR[1];
        constexpr uint32_t read_errors = DSI_ISR1_GPRXE | DSI_ISR1_GPRDE | DSI_ISR1_PSE | DSI_ISR1_CRCE |
                                         DSI_ISR1_ECCME | DSI_ISR1_TOLPRX;
        return (e1 & read_errors) != 0u ? DsiStatus::error : DsiStatus::ok;
    }

    /// DCS, the spelling a panel understands: a command alone (the short
    /// packet with no parameter), with one parameter (the short packet
    /// with one), with more (the long packet, the command its first
    /// byte).
    static DsiStatus dcs_write(uint8_t command) {
        return short_write(DsiDataType::dcs_short_write_0, command, 0u);
    }
    static DsiStatus dcs_write(uint8_t command, uint8_t parameter) {
        return short_write(DsiDataType::dcs_short_write_1, command, parameter);
    }
    static DsiStatus dcs_write(uint8_t command, const uint8_t* parameters, uint16_t count) {
        if (count == 0u) {
            return dcs_write(command);
        }
        if (count == 1u) {
            return dcs_write(command, parameters[0]);
        }
        if (count > 255u) {
            return DsiStatus::refused;
        }
        uint8_t payload[256];
        payload[0] = command;
        for (uint16_t i = 0; i < count; ++i) {
            payload[i + 1u] = parameters[i];
        }
        return long_write(DsiDataType::dcs_long_write, payload, static_cast<uint16_t>(count + 1u));
    }
    /// A DCS read: the command's answer, `len` bytes of it.
    static DsiStatus dcs_read(uint8_t command, uint8_t* buf, uint16_t len) {
        return read(DsiDataType::dcs_read, command, 0u, buf, len);
    }

    /// A generic write of 0, 1, 2 or more bytes.
    static DsiStatus generic_write(const uint8_t* data, uint16_t count) {
        switch (count) {
            case 0: return short_write(DsiDataType::generic_short_write_0, 0u, 0u);
            case 1: return short_write(DsiDataType::generic_short_write_1, data[0], 0u);
            case 2: return short_write(DsiDataType::generic_short_write_2, data[0], data[1]);
            default: return long_write(DsiDataType::generic_long_write, data, count);
        }
    }
    /// A generic read with 0, 1 or 2 parameters.
    static DsiStatus generic_read(const uint8_t* params, uint8_t count, uint8_t* buf, uint16_t len) {
        switch (count) {
            case 0: return read(DsiDataType::generic_read_0, 0u, 0u, buf, len);
            case 1: return read(DsiDataType::generic_read_1, params[0], 0u, buf, len);
            case 2: return read(DsiDataType::generic_read_2, params[0], params[1], buf, len);
            default: return DsiStatus::refused;
        }
    }

    /// The generic interface's FIFO flags (GPSR).
    static bool command_fifo_empty() { return (regs().GPSR & DSI_GPSR_CMDFE) != 0u; }
    static bool write_fifo_empty() { return (regs().GPSR & DSI_GPSR_PWRFE) != 0u; }
    static bool read_fifo_empty() { return (regs().GPSR & DSI_GPSR_PRDFE) != 0u; }
    static bool read_busy() { return (regs().GPSR & DSI_GPSR_RCB) != 0u; }

    // ---- errors and interrupts (18.13) -------------------------------------------

    /// DSI_ISR0 and DSI_ISR1, cleared by the read that returns them (item 5).
    static uint32_t errors0() { return regs().ISR[0]; }
    static uint32_t errors1() { return regs().ISR[1]; }
    /// DSI_IER0 and DSI_IER1: which error bits raise the vector.
    static void error_interrupts(uint32_t mask0, uint32_t mask1) {
        regs().IER[0] = mask0;
        regs().IER[1] = mask1;
    }
    /// DSI_FIR0 and DSI_FIR1: the errors forced, for a handler's test.
    static void force_errors(uint32_t mask0, uint32_t mask1) {
        if (mask0 != 0u) {
            regs().FIR[0] = mask0;
        }
        if (mask1 != 0u) {
            regs().FIR[1] = mask1;
        }
    }

    /// DSI_WIER's five enables, as a mask of DSI_WISR bits.
    static void wrapper_interrupts(uint32_t wisr_mask) {
        regs().WIER = wisr_mask & (DSI_WIER_TEIE | DSI_WIER_ERIE | DSI_WIER_PLLLIE | DSI_WIER_PLLUIE | DSI_WIER_RRIE);
    }
    static uint32_t wrapper_flags() { return regs().WISR; }
    /// DSI_WIFCR: the five flags share the WISR bit positions.
    static void clear_wrapper_flags(uint32_t wisr_mask) {
        regs().WIFCR = wisr_mask & (DSI_WISR_TEIF | DSI_WISR_ERIF | DSI_WISR_PLLLIF | DSI_WISR_PLLUIF | DSI_WISR_RRIF);
    }

    /// The ISR body for the one vector: the wrapper's five flags taken and
    /// cleared, both error registers read - which clears them.
    [[gnu::always_inline]] static DsiEvents isr() {
        DsiEvents e{};
        const uint32_t w = regs().WISR;
        const uint32_t flags = w & (DSI_WISR_TEIF | DSI_WISR_ERIF | DSI_WISR_PLLLIF | DSI_WISR_PLLUIF | DSI_WISR_RRIF);
        if (flags != 0u) {
            regs().WIFCR = flags;
        }
        e.tearing_effect = (w & DSI_WISR_TEIF) != 0u;
        e.end_of_refresh = (w & DSI_WISR_ERIF) != 0u;
        e.pll_locked = (w & DSI_WISR_PLLLIF) != 0u;
        e.pll_unlocked = (w & DSI_WISR_PLLUIF) != 0u;
        e.regulator_ready = (w & DSI_WISR_RRIF) != 0u;
        e.errors0 = regs().ISR[0];
        e.errors1 = regs().ISR[1];
        return e;
    }

    static void enable_interrupt() { Nvic::enable(irq); }
    static void disable_interrupt() { Nvic::disable(irq); }

    // ---- the words, for a test to check against the manual --------------------------

    /// GHCR: the type in bits 5:0, the virtual channel in 7:6, the two
    /// bytes above (18.15.24).
    static uint32_t header(DsiDataType dt, uint8_t p0, uint8_t p1) {
        return (static_cast<uint32_t>(dt) & DSI_GHCR_DT_Msk) |
               ((static_cast<uint32_t>(regs().GVCIDR) & DSI_GVCIDR_VCID_Msk) << DSI_GHCR_VCID_Pos) |
               (static_cast<uint32_t>(p0) << DSI_GHCR_WCLSB_Pos) | (static_cast<uint32_t>(p1) << DSI_GHCR_WCMSB_Pos);
    }

    /// The thirteen transmission-type bits of CMCR, "low power" when set:
    /// the eleven of the short packets and the maximum read size, and the
    /// two of the long writes - apart, because the frame of the adapted
    /// command mode goes out as DCS long writes and wants high speed
    /// while a panel's configuration may stay in low power.
    static constexpr uint32_t short_type_bits =
        DSI_CMCR_GSW0TX | DSI_CMCR_GSW1TX | DSI_CMCR_GSW2TX | DSI_CMCR_GSR0TX | DSI_CMCR_GSR1TX | DSI_CMCR_GSR2TX |
        DSI_CMCR_DSW0TX | DSI_CMCR_DSW1TX | DSI_CMCR_DSR0TX | DSI_CMCR_MRDPS;
    static constexpr uint32_t long_type_bits = DSI_CMCR_GLWTX | DSI_CMCR_DLWTX;
    static constexpr uint32_t command_type_bits = short_type_bits | long_type_bits;

    /// ODF's code: log2 of the divisor (18.16.11).
    static constexpr uint8_t odf_code(uint8_t odf) {
        return odf == 8u ? 3u : odf == 4u ? 2u : odf == 2u ? 1u : 0u;
    }

private:
    static bool wait(const volatile uint32_t& reg, uint32_t mask, bool set, uint32_t spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (((reg & mask) != 0u) == set) {
                return true;
            }
        }
        return false;
    }
    static void bit(volatile uint32_t& reg, uint32_t mask, bool on) {
        if (on) {
            reg |= mask;
        } else {
            reg &= ~mask;
        }
    }
};

#endif   // DSI_BASE

}   // namespace brio
