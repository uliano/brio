# DSI Host (STM32F4)

Documents of record: RM0386 Rev 6 ch. 18 (the F469/F479's, the only
class of the family with the block - 18.4 the host, 18.6 the adapted
command mode, 18.11 the pattern generator, 18.12 the D-PHY, its PLL and
its regulator, 18.13 the interrupts and the error table, 18.14 the
programming procedure, 18.15 and 18.16 the host's and the wrapper's
registers); its RCC chapter for the clock tree (figure 17: the PLL off
HSE, the lane byte clock an eighth of the bit rate, DCKCFGR's DSISEL);
DS11189 Rev 8 5.3.13 .. 5.3.15 (tables 45 .. 48: the unit interval, the
LP/HS transition times, the PLL's and the regulator's ranges) and its
table 12 for the tearing effect pad; ES0321 Rev 14 2.8.1 .. 2.8.3; UM1932
Rev 5 4.14 for what the board wires around its panel. For the panel the
bench suite is measured against, the two controllers the MB1166 was
built with: Novatek's NT35510 (the one this unit answers as: its
datasheet documents the DSI as a command interface - the frame memory
written by RAMWR/RAMWRC and read by RAMRD/RAMRDC, the standard DCS set,
RDID1 .. 3 - and names a Manufacture Command Set it does not describe)
and Orise's OTM8009A (preliminary 0.92: the same DCS set, its ID bytes,
and table 6.4.2.1 for the porches the frame uses). The packet data types
are the MIPI DSI specification's, which is not on the desk: the codes in
`DsiDataType` are its table 16 as every DSI peripheral shares them, and
both panel datasheets draw that table as a picture. Driver:
`stm32f4/dsi.hpp` (`Dsi`, `DsiConfig` and its parts `DsiPllConfig`,
`DsiPhyConfig`, `DsiPhyTiming`, `DsiHostConfig`, `DsiVideoConfig`,
`DsiVideoTiming`, `DsiColor`, `DsiLanes`, `DsiVideoType`, `DsiPattern`,
`DsiDataType`, `DsiStatus`, `DsiEvents`, `DsiLaneStatus`, and the
arithmetic `dsi_pll_for`, `dsi_config_for`, `dsi_uix4`,
`dsi_lane_byte_hz`, `dsi_escape_divider`, `dsi_phy_timing_for`,
`dsi_video_timing`, `dsi_line_fits`), over the reserve's `dsi_present`,
`dsi_clock_mask`, `dsi_reset_mask`, `dsi_byte_clock_select_mask` and
`dsi_irq` (`stm32f4/device_tables.hpp`). The family fixture is
`test/family_stm32f4/dsi.cpp` with the negatives that refuse the type on
a part with no block and a PLL triple outside the ranges. Bench:
`test_stm32f4_dsi` on the 32F469IDISCOVERY.

## What the silicon does

**Three things under one register file and one vector.** The DSI
WRAPPER (18.16) holds the 1.2 V regulator that powers the D-PHY, the
PLL that clocks it, the LTDC's pixel stream turned into the host's
input and the tearing effect line; the DSI HOST (18.15) is the packet
handler, the video mode state machine, the APB generic interface a
program writes packets through and the error registers; the D-PHY is
one clock lane and two data lanes on dedicated pads, each in high-speed
differential signalling or low-power single-ended signalling. Two ways
to a panel: VIDEO MODE, where the LTDC's frame is packed into pixel
packets and streamed at the panel's timing; and the ADAPTED COMMAND
MODE (18.6), where each LTDC frame becomes DCS memory writes - a
write_memory_start then write_memory_continue packets of LCCR.CMDSIZE
pixels each - that a panel with a frame memory takes at its own pace.
The generic interface works beside either: a DCS command goes out in
low-power signalling, a read turns the bus around and the panel's
answer comes back on data lane 0. Two of the pack's twenty-three headers
declare the block; the F469 and the F479 have it and the rest have
nothing.

**The panel decides between the two ways.** A DSI panel whose interface
is a command one - the 32F469IDISCOVERY's NT35510, whose datasheet
documents RAMWR/RAMWRC and RAMRD/RAMRDC and nothing of a video stream -
shows nothing of video mode: measured, a second of video-mode frames and
the pattern generator's bars leave its frame memory untouched (a pixel
read back before and after is the same) while its power mode and its
error count stay clean, and one refresh of the adapted command mode puts
the frame there. What the panel then holds can be READ over the same
link: RAMRD answers three bytes a pixel in the 24-bit format, the pixel
first with no leading byte, at the column and row CASET and RASET
point to - which is what makes the picture a measurement.

**A refresh is one LTDC frame** (18.6): WCR.LTDCEN lets the LTDC run,
the wrapper packs its lines into DCS long writes, halts it on the VSync
edge WCFGR.VSPOL names, clears LTDCEN and raises ERIF; WCFGR.AR launches
one per tearing effect pulse instead. The memory writes are DCS long
writes, so the CMCR bit that decides their signalling (DLWTX) is the
frame's speed: `DsiHostConfig` keeps the long writes' two bits apart
from the short commands' eleven, and a refresh at 496 Mbit/s takes
exactly the LTDC's 16.6 ms frame - the link is faster than the pixel
clock and the wrapper's flow control keeps them in step.

**The panel's receiver locks onto the clock lane's entry into high
speed.** Measured: a panel reset with the clock lane already running in
high speed took no high-speed packet afterwards - every frame refreshed
into it was lost while every low-power command and read went through,
and neither side counted an error - until the host was disabled and
enabled again, which restarts the clock lane. So the bring-up keeps the
clock lane in low power (`DsiPhyConfig::clock_lane_hs` false) through
the panel's reset and its low-power configuration and brings it to high
speed after (`clock_lane(true)`); with that order the first frame lands
and the re-enables that follow report nothing at the panel's next
turnaround.

**The PLL's ranges are stated twice, and not the same way.** RM0386
18.12.5: CLKIN 4..100 MHz, the input after IDF 8..50 MHz, the VCO
500..1000 MHz, the output 31.25..500 MHz, NDIV 10..125, IDF 1..7, ODF 1,
2, 4 or 8. DS11189 table 47: the same, but the PFD input 4..25 MHz. The
solver keeps to the intersection, 8..25 MHz. Off the board's 8 MHz
crystal that makes 496 Mbit/s exact (IDF 1, NDIV 62, ODF 1, the VCO at
992 MHz) and 500 not (it would want IDF 2 and a 4 MHz PFD); a 25 MHz
crystal reaches 500 with NDIV 20. The output IS the bit rate of one
lane: the RCC's figure 17 draws the 500 MHz "high speed clock" whose
eighth is the lane byte clock the host counts in, and DS11189 table 45's
unit interval of 2..12.5 ns is the same range, 80..500 Mbit/s. CLKIN is
HSE and nothing else.

**The unit interval is the one D-PHY field there is no default for**
(18.14.2): WPCR0.UIX4, the bit period in quarter nanoseconds, rounded
DOWN - 8 at 496 Mbit/s, the manual's own example 6 at 600. Every other
D-PHY timing the wrapper lets a program override (WPCR1 .. WPCR4) is
derived from it inside the PHY and left at its default.

**The host counts in lane byte clocks.** HSA, HBP and the line
(DSI_VHSACR, VHBPCR, VLCR) are the LTDC's pixel counts at the ratio of
the lane byte clock to the pixel clock, ROUNDED (18.14.6) - at 62 MHz
over 26.4 MHz, 2 pixels of sync are 5 cycles, 20 of porch 47, the 842 of
a line 1977. The rounding accumulates line after line unless the host
returns to low power once a line and resynchronises on the next sync
event, which is why `DsiVideoConfig` allows every return by default. The
LP/HS transition times of the lanes (DSI_CLTCR, DSI_DLTCR) are lane byte
clocks too, computed from DS11189 table 46's maxima with half again of
margin: clock lane LP to HS 50 + 300 + 8 UI ns, HS to LP 62 + 52 UI +
60 + 100 ns, data lane LP to HS 50 + 145 + 10 UI ns, HS to LP 60 + 8 UI
+ 100 ns - 35, 31, 21 and 17 cycles at 496 Mbit/s.

**Configuration happens with the host disabled.** DSI_CR.EN = 0 holds
the host "under reset" (18.15.2) - and measured, its registers keep
their values through it: the same PCONFR, CMCR, PCR, LCOLCR, LCCR and
CLTCR after a disable, a PLL and regulator cycle and an enable. WCFGR's
fields and WPCR1 .. 4 are documented as writable only with both
DSI_CR.EN and DSI_WCR.DSIEN clear, and the programming procedure of
18.14.1 writes everything first and enables last. Every configuring
verb refuses while enabled, and writes nothing then; a program changes
mode with the host disabled around the change.

**In video mode the generic interface lives inside the stream.** The
procedure of 18.14.1 lists the DCS commands (step 16) before the LTDC's
enable (step 17); measured, a read issued in video mode with the LTDC
stopped never completes: the request leaves the command FIFO, GPSR.RCB
stands, and the answer arrives with the first frame after the LTDC
starts (the same read, issued then, is answered). In command mode the
same read is answered with nothing else on the link - no stream, no
refresh in flight - which is what makes a panel's bring-up in that mode
the plain one.

**A read is two packets and a turn-around** (18.15.24 .. 18.15.26). The
host has no read verb: the program sets the maximum return packet size
(data type 0x37, its two bytes the size), sends the read request, and
the panel takes the bus (DSI_PCR.BTAE must be set) and answers in
low-power signalling on lane 0; GPSR.RCB stands until the whole response
is in the read FIFO, and the payload comes out of DSI_GPDR a word at a
time with the header and its checks stripped. A response nobody
collected - a read that ran out and was answered later - would be handed
to the next read, so `read()` drains the FIFO before it starts.

**The host's error registers clear on the read that shows them**
(18.13.2), and the acknowledge-with-error bits are the PANEL'S report,
delivered at the next bus turnaround - so a read is what makes them
appear, and they describe what happened since the previous read.
Measured on the NT35510: the first turnaround after the host's disable
and enable carries AE6 (a false control error: the lanes moved while its
receiver watched) in command mode and under a running video stream, and
AE13 (an invalid transmission length) in video mode with the LTDC
stopped; the first after the panel's own reset carries AE6 when the host
was streaming and nothing when it was not. One report each, nothing
follows, and nothing of it is a link that failed: a program that
re-enables the host reads the registers once after its first exchange.

**Three errata, two of them code** (ES0321 Rev 14):
- *2.8.2, incorrect calculation of the time to activate the clock
  between HS transmissions*: with the automatic clock lane control the
  host uses twice HS2LP_TIME instead of HS2LP + LP2HS. `phy()` writes
  both clock-lane fields with the larger of the two values, whether or
  not ACR is on - the register reads 35 and 35.
- *2.8.3, the immediate update procedure may fail*: VSCR's UR and EN set
  in one write may race. `shadow_update()` writes 0 then 0x101 and
  checks UR auto-cleared, repeating while it has not.
- *2.8.1, tearing effect parasitic detection*: the tearing effect over
  the LINK raises TEIF on every acknowledge trigger. Not code:
  `te_source()` offers the dedicated pin, and the suite measures the
  wrapper's flag from PJ2 pulse for pulse against the EXTI's count of the
  same edges, and the automatic refresh paced by it.

**Three small facts the registers taught**, none of them a workaround:
DSI_PSR wakes as 0x1400 and not the manual's 0x1528 (its defined bits
all clear with the regulator off, two reserved ones set); clearing
WRPCR.PLLEN drops PLLLS and raises NO PLLUIF, the unlock flag being a
lost lock's and not a software stop's; and a store into DSI_FIR shows in
DSI_ISR a moment later, not on the read straight after it.

## Types and verbs

- `DsiDataType`: the packet data types a processor sends (MIPI DSI 1.1
  table 16) - the sync events, the colour mode and shutdown packets, the
  generic and DCS short writes and reads, `set_maximum_return_packet_size`,
  the long writes, the four pixel streams. A DCS program never spells
  one.
- The limits as constants: `dsi_bit_rate_min_hz`/`max_hz` (80..500
  Mbit/s), `dsi_lane_byte_max_hz` (62.5 MHz), `dsi_escape_clock_max_hz`
  (20 MHz), the PLL's `dsi_pll_in/pfd/vco/out_min/max_hz`.
- `DsiPllConfig` (`ndiv`, `idf`, `odf`; `ndiv == 0` is none) with
  `dsi_pll_vco_hz`, `dsi_pll_bit_rate_hz`, `dsi_pll_config_ok(clkin,
  c)` and `dsi_pll_for(clkin, bit_rate)`, the exact search (the smallest
  IDF first). `dsi_lane_byte_hz(bit_rate)`, `dsi_uix4(bit_rate)`,
  `dsi_escape_divider(lane_byte_hz)` (never below 2: 0 and 1 stop the
  clock), `dsi_cycles_for_ns`.
- `DsiPhyTiming` (the four transition times, `max_read_time`,
  `stop_wait`) with `dsi_phy_timing_ok` and `dsi_phy_timing_for(bit_rate)`;
  `DsiPhyConfig` (`lanes`, `clock_lane_hs`, `automatic_clock_lane`,
  `timing`).
- `DsiHostConfig`: the escape and timeout dividers, the five flow
  control bits (`bus_turn_around` on by default - a read needs it),
  `commands_low_power` for the eleven short-packet bits and
  `long_writes_low_power` for the two long-write ones (a refresh's
  memory writes go in high speed), the two acknowledge requests, the
  virtual channel, the seven timeout counters (0 = none), the two
  largest-LP-packet sizes; `dsi_host_config_ok`.
- `DsiColor` (the six codes of LCOLCR.COLC and WCFGR.COLMUX),
  `DsiVideoType` (sync pulses, sync events, burst), `DsiVideoConfig`
  (the type, the six low-power returns, `lp_commands`, `frame_bta_ack`,
  `loosely_packed`, `packet_pixels`, `chunks`, `null_bytes`),
  `DsiVideoTiming` (HSA, HBP and the line in lane byte clocks, the
  vertical four in lines, the width) with `dsi_video_timing_ok`,
  `dsi_lane_cycles`, `dsi_video_timing(ltdc_timing, pixel_hz,
  lane_byte_hz)`, `dsi_line_payload_bytes` and `dsi_line_fits(v, colour,
  lanes)`; `DsiPattern` (off, vertical bars, horizontal bars, BER).
- `DsiConfig` (the PLL, the bit rate beside it, the D-PHY, the host)
  with `dsi_config_ok(clkin, c)` and `dsi_config_for(clkin, bit_rate)`,
  which solves the PLL and derives the timings and the escape divider.
- `DsiStatus` (`ok`, `refused` with nothing sent, `timeout`, `error`),
  `DsiEvents` (the wrapper's five flags and both error registers),
  `DsiLaneStatus` (DSI_PSR decoded: the stop and ULPS state of each lane,
  the direction).
- `Dsi`, the monostate over the block where the header declares it:
  `clock`, `reset_block`, `version`, `byte_clock_from_pllr` (read only:
  the clock task fixes PLLR at 2, above the lane byte clock's ceiling
  at every rate this stratum runs); `regulator(on)` waited for on RRS and
  `regulator_ready`; `pll(clkin, c)` (stopped, written, started, waited
  for on PLLLS; refused outside the ranges), `pll_off`, `pll_on`,
  `pll_locked`, `pll_config`; `phy(c, bit_rate)` (UIX4, the lanes and
  the stop wait, the clock lane's mode, the transition times with 2.8.2
  applied), `uix4`, `lanes`, `phy_timing`, `lane_status`,
  `ulps(clock, data, enter)`, `clock_lane(high_speed)` (CLCR.DPCC alone,
  live) and its readback; `host(c)`, `escape_divider`,
  `commands_low_power`, `long_writes_low_power`, `virtual_channel`;
  `colour(c, ltdc_timing, loosely)` (LCOLCR and WCFGR.COLMUX together,
  LPCR's polarities from the LTDC's own words), `te_source(from_pin,
  falling)`, `adapted_refresh(automatic, halt_on_rising)`;
  `command_mode(pixels)` (MCR.CMDM, WCFGR.DSIM and LCCR.CMDSIZE) and
  `video(cfg, timing)` (refused while enabled, outside the fields, or
  when a line's pixels do not fit its lane byte clocks on the lanes
  configured), `in_command_mode`, `video_timing`, `pattern(p)`,
  `pattern_on`, `shadow_update` (2.8.3), `shadow`, `shadowed`;
  `init<cfg, clkin>()` (static_assert) and `init(clkin, cfg)` - the gate,
  the host disabled, the regulator, the PLL, the D-PHY, the host's
  timings - then `enable()` (DEN, CKE, EN, DSIEN in the procedure's
  order), `disable()`, `enabled()`, `release()`; the wrapper's
  `ltdc_flow` (a refresh's trigger in the adapted command mode, cleared
  by the wrapper at its end), `shutdown`, `eight_colours`, `busy`; the
  generic interface `short_write(dt, p0, p1)`, `long_write(dt, payload,
  len)`, `read(dt, p0, p1, buf, len)`, and over them `dcs_write(cmd)`,
  `dcs_write(cmd, param)`, `dcs_write(cmd, params, n)`, `dcs_read(cmd,
  buf, n)`, `generic_write`, `generic_read`, with the FIFO flags
  `command_fifo_empty`, `write_fifo_empty`, `read_fifo_empty`,
  `read_busy`; `errors0`/`errors1` (read-and-clear by the silicon's
  design), `error_interrupts`, `force_errors`, `wrapper_interrupts`,
  `wrapper_flags`, `clear_wrapper_flags`, the ISR body `isr()`,
  `enable_interrupt`/`disable_interrupt`; `header(dt, p0, p1)`,
  `short_type_bits`, `long_type_bits` and `command_type_bits` for a
  test to check against the manual.
- The reserve's facts: `dsi_present()`, the APB2 gate and reset masks,
  `dsi_byte_clock_select_mask()`, `dsi_irq()`.

## How to use it

The panel's words stay with the panel: which command wakes it, sets its
format, its write window or its backlight is its datasheet's, spelled
beside the program that talks to it. The link's numbers are the
program's choice of bit rate and the LTDC's timing.

```cpp
#include "stm32f4/dsi.hpp"
#include "stm32f4/ltdc.hpp"
using namespace brio;

constexpr LtdcTiming panel{.hsync = 2, .hbp = 20, .width = 800, .hfp = 20,
                           .vsync = 10, .vbp = 15, .height = 480, .vfp = 16};
constexpr DsiConfig link = [] {                                   // off the 8 MHz crystal
    DsiConfig c = dsi_config_for(8'000'000u, 496'000'000u);
    c.host.long_writes_low_power = false;                         // the frame in high speed
    c.phy.clock_lane_hs = false;                                  // low power until the panel is up
    return c;
}();

// the memory and the LTDC first (fmc.md, ltdc.md): the pixel clock, the
// timing, a layer over a frame buffer, the controller enabled - the
// wrapper holds it until a refresh
Dsi::init<link, 8'000'000u>();             // regulator, PLL, D-PHY, host
Dsi::colour(DsiColor::rgb888, panel);      // the LTDC interface
Dsi::command_mode(800);                    // a line per memory write
Dsi::te_source(true);                      // the tearing effect from the pin
Dsi::adapted_refresh(false, false);        // manual refreshes, the LTDC halted on VSYNC's fall
Dsi::enable();

// the panel, in its own words - a reset on the board's pad, then DCS
Dsi::dcs_write(0x11);                      // SLPOUT
// ... 120 ms ...
Dsi::dcs_write(0x3A, 0x77);                // COLMOD, 24 bits
Dsi::dcs_write(0x36, 0x60);                // MADCTL, landscape
Dsi::dcs_write(0x2A, columns, 4);          // CASET, 0..799
Dsi::dcs_write(0x2B, rows, 4);             // RASET, 0..479
Dsi::dcs_write(0x29);                      // DISPON
uint8_t id[3];
Dsi::dcs_read(0xDA, &id[0], 1);            // RDID1: the controller answers
(void)Dsi::errors0();                      // the first turnaround's report, read once
Dsi::clock_lane(true);                     // THEN the clock lane into high speed

// a frame: paint the surface, launch, wait for the end of refresh
Dsi::clear_wrapper_flags(DSI_WISR_ERIF);
Dsi::ltdc_flow(true);
while ((Dsi::wrapper_flags() & DSI_WISR_ERIF) == 0u) {
}
```

A program that streams to a panel whose DSI takes video packets
configures `video(cfg, dsi_video_timing(panel, pixel_hz,
dsi_lane_byte_hz(bit_rate)))` instead of `command_mode()`, starts the
LTDC and then talks to the panel - in video mode a command is sent
inside the stream. An interrupt-driven program binds `DSI_IRQHandler` to
a body that calls `Dsi::isr()`, arms `wrapper_interrupts()` for the five
wrapper events (the end of refresh among them) and `error_interrupts()`
for the error bits it wants the vector on; the body clears the wrapper's
flags and reads both error registers.

## Bench findings

`test_stm32f4_dsi` on the 32F469IDISCOVERY (STM32F469NI at 180 MHz,
PCLK2 90 MHz, the IS42S32400F on FMC bank 1 at 90 MHz holding the frame
buffers, the LTDC at a 26.4 MHz pixel clock, the link at 496 Mbit/s on
two lanes) against the MB1166 panel, reset on PH7, tearing effect on
PJ2: **ALL: 92 pass, 0 fail**, twice - and two letters by name that want
a human, one of them with the module's touch controller on I2C1.

- **The module's controller is Novatek's NT35510**: RDID1 00h, RDID2
  80h, RDID3 00h over the link, the same three bytes sixteen times
  running and after every excursion of the suite - the MB1166's
  revision is not readable on this unit (the display board is captive
  over the main board), and the ID is what says which of the two
  controllers it carries. Its power mode reads 9Ch out of the bring-up
  (booster on, sleep out, normal, display on), its own count of DSI
  errors 0 after every letter.
- **The picture is in the panel, and read back**: in the adapted
  command mode one refresh of 800x480 32-bit pixels takes 16617 us -
  the LTDC's frame is 16616 - and ends with ERIF, LTDCEN cleared by the
  wrapper and BUSY down; refreshes back to back make 61 frames a second
  with no payload write error, no FIFO underrun and the panel's error
  count at 0; sixteen pixels read back through RAMRD at the corners, the
  middle and inside every bar are the bars the accelerator painted, 24
  bits each, the answer starting with the pixel (no leading byte); a run
  of sixteen across the white/yellow boundary reads back in one packet;
  the 16-bit surface's yellow (565 FFE0h) reaches the panel as FFFF00h,
  the LTDC's expansion replicating the high bits. A human sees the eight
  bars, and a bar sliding along the bottom at one refresh a step, 179
  steps in three seconds.
- **The first frame after the bring-up**: with the clock lane brought to
  high speed only after the panel's reset and configuration, the frame
  refreshed at boot is in the panel's memory before any letter runs
  (sixteen pixels read back right after boot); with the clock lane in
  high speed through the panel's reset, the same frame was lost with no
  error anywhere and came back only after the host's disable and enable.
- **The module's touch frame on its display** (the letter by name that
  wants a finger, with the FocalTech controller read over I2C1 as the
  I2C suite reads it): a white square refreshed into the panel at ten
  random places, each tap's raw coordinates recorded against the
  square's centre, and the orientation solved from them - of the eight
  axis swaps and mirrors, x = ty and y = 479 - tx fits, the controller's
  frame being the module's portrait one; 21..24 px rms over ten taps and
  40..43 px at worst, a fingertip's width on this glass, with the red
  cross drawn where each tap was understood landing under the finger.
- **This panel's DSI is a command interface**: the 16-bit bars streamed
  in video mode for a second (60 frames, no payload error, no underrun,
  a DCS read answered in a blanking period, the panel's error count 0)
  and the pattern generator's bars after them leave the panel's pixel
  (0, 0) exactly as the refresh before had written it. In video mode
  with the LTDC stopped RDID1 runs out - GPSR 0x55, RCB standing with
  the command FIFO empty - and the answer comes with the first frame
  after the LTDC starts (GPSR 0x05, the read FIFO holding it); in command
  mode the same read is answered with nothing else on the link.
- **The tearing effect line and the automatic refresh**: with TEON 30
  rising edges on PJ2 in half a second on EXTI line 2, and 30 TEIF flags
  in the wrapper from the same pin (WCFGR.TESRC); with WCFGR.AR set the
  wrapper made 59 refreshes in a second against 59 TE edges, no error; 0
  edges after TEOFF.
- **The block at reset**: the APB2 gate closed; VR 0x3133302A, CR 0, MCR
  1 (command mode), PCONFR 1 (two lanes), GPSR 0x15 (every FIFO empty),
  PSR 0x1400 (not the manual's 0x1528: the defined bits clear with the
  regulator off), WRPCR, WISR, WCFGR, VMCR, CLTCR and DLTCR all 0.
- **The regulator and the PLL**: the regulator ready in 57..102 us, the
  PLL locked in 68 us (tLOCK 200 us at most) at boot and at every relock;
  PLLEN cleared drops PLLLS and raises no PLLUIF; a triple with a 4 MHz
  PFD is refused with PLLEN left clear; REGEN cleared takes RRS down and
  set brings it back with RRIF; the host's six configuration registers
  read the same after the whole excursion, the panel answers its
  identity again and takes a frame.
- **What the driver refuses**: the seven configuring verbs while
  enabled, none of them writing a register; a line that holds no pixels,
  one the pixels do not fit (1200 lane byte clocks for 1203 of payload a
  lane plus 52 of sync and porch), a field past its bits, a chunk count
  past thirteen bits, an escape divider that stops the clock; a write
  and a read while disabled, a read of zero bytes; and both modes taken
  again with the host disabled.
- **The D-PHY at rest**: with no refresh in flight both data lanes in
  stop state, neither in ULPS, the direction ours, the clock lane NOT in
  stop state (DPCC keeps it in high speed); the three FIFOs empty, no
  read and no refresh in flight; no acknowledge error along the
  bring-up nor at the first read after the suite's re-enables of the
  host - the AE6 those reads found under the earlier order, with the
  clock lane in high speed through the panel's reset, is gone with it.
- **The panel's registers**, each through its read-back command: COLMOD
  55h and 77h read 5 and 7 (RDDCOLMOD's low three bits); MADCTR 00h and
  60h read back as written; TEON and TEOFF in RDDSM's bit 7; WRDISBV
  reads back A FRAME LATER (80h reads FFh straight after the write and
  80h 40 ms on), WRCTRLD's BCTRL and BL as written; DISPOFF and DISPON in
  RDDPM's DISON, SLPIN and SLPOUT in its SLPOUT 120 ms apart; the panel's
  error count 0 after all of it.
- **The errors and the vector**: a forced error shows in ISR0 a moment
  after the store and the read that shows it clears it, the same in ISR1;
  AE1 enabled and forced enters the vector once with the body reporting
  it, AE2 forced unmasked enters nothing and stands until read; WIFCR
  clears the lock and unlock flags, the PLL's relock enters the vector
  with PLLLIF, and the end of a refresh with ERIF, once.
- **The wrapper's packets**: WCR.SHTDN and its clear go out with no
  error on either side, and this controller does not act on them (RDDPM
  9Ch throughout); WCR.COLM the same.
- **ULPS**: PUCR.URDL takes both data lanes into the ultra-low-power
  state (UAN0 and UAN1 fall, stop state left), UEDL brings them back to
  stop state, the panel answers after it and takes a frame.

## Not covered yet

Driver gaps:
- **Video mode into a panel that takes it**: the host's side is measured
  (the frame at the pixel clock's rate, the pattern generator, the
  errors) and the panel's is not - this module's controller is a
  command interface, and a video-mode panel, or this one's Manufacture
  Command Set (a document not on the desk, which the NT35510's datasheet
  names and does not describe), would measure the picture that way.
- **Burst mode and the chunked line** (VMT 1x, NUMC, NPSIZE): the fields
  are written and checked and no burst frame was streamed; a panel that
  wants its line compressed would measure it.
- **The tearing effect over the link** (WCFGR.TESRC = 0, CMCR.TEARE):
  declined, ES0321 2.8.1 - the pin is measured, pulse for pulse.
- **The lane byte clock off the main PLL's R output** (DCKCFGR.DSISEL):
  the clock task fixes PLLR at 2, whose output is above 62.5 MHz at every
  rate this stratum runs, so the selector is read and never written.
- **The timeout counters** (TCCR0 .. TCCR5) and the timeout errors: left
  at 0 - a panel that misbehaves on the bench would give them a letter.
- **The D-PHY's custom timings** (WPCR1 .. WPCR4: the slew rates, the
  delays, the tHS/tCLK overrides): the defaults, derived from UIX4, drove
  this panel; a link with a marginal layout would want them exposed.
- **One data lane** (PCONFR.NL = 0): the enumerator exists and the board
  wires two.
- **A partial refresh** (a window smaller than the frame through CASET,
  RASET and the LTDC's layer window): the whole frame is what the suite
  refreshes; a program updating a region would measure the wrapper's
  packing of a shorter line.
- **The panel's own controller beyond DCS**: the NT35510's manufacturer
  command pages and the OTM8009A's CMD2 registers are the panels', not
  this chapter's; the suite drives the module with the standard set
  alone, which this module takes.

Implemented, not bench-verified:
- `eight_colours` and `shutdown` as the PANEL sees them: the packets go
  out without an error and this controller ignores them; a panel that
  acts on the shutdown-peripheral packet would show it in its power mode.
- `automatic_clock_lane` (CLCR.ACR): the bit is written, the erratum's
  rule is applied in either case, and the link ran with the clock lane
  continuous; the power it would save is what a meter on VDD12DSI would
  measure.
- `shadow_update` and `shadow`: the verbs write their registers; the
  shadow copies serve a video frame in flight, and no configuration was
  changed under one.
- `frame_bta_ack`, `acknowledge_request` and `te_acknowledge`: the bits
  are written and never set - the panel's acknowledges arrived on the
  reads' own turnarounds.
- `generic_write` and `generic_read`: the same packets as the DCS verbs
  with other data types; this panel speaks DCS.
- The BER pattern (`DsiPattern::ber`) and the loosely packed 18-bit
  coding: written, not streamed - an oscilloscope on the lanes, or a
  panel that takes 18 bits, would.
- The clock lane's ULPS (PUCR.URCL/UECL): the data lanes were measured;
  the clock lane runs continuously here.
- `WCFGR.VSPOL` on its rising edge: the LTDC was halted on the falling
  edge of its active-low VSYNC throughout.
