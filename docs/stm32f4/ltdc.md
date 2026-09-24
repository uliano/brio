# LTDC, the LCD-TFT display controller (STM32F4)

Documents of record: RM0090 Rev 22 ch. 16, whose first line is "this
section applies only to" the parts with a display interface - RM0390
Rev 6 and RM0383 Rev 4 have no such chapter, because the F446 and the
F411 have no such peripheral. The pixel clock's PLL is RM0090 6.3.24
(RCC_PLLSAICFGR) and 6.3.25 (RCC_DCKCFGR's PLLSAIDIVR). Errata: ES0206
Rev 24 has SIXTEEN sections and NOT ONE of them is this peripheral's -
the display controller and the accelerator beside it are the only two
chapters of this stratum with no erratum at all. Datasheet DS10693
table 12 is where the alternate function of each signal is stated (AF14
for every one of them except four pads that carry theirs on AF9) and
DS10693 table 107 is where the pixel clock's ceiling is: 83 MHz.
Driver: `stm32f4/ltdc.hpp`; the presence facts are in
`stm32f4/device_tables.hpp` and the PLL's verbs in `stm32f4/clock.hpp`.
Bench suite: `test_stm32f4_ltdc`, against the 240x320 panel the
STM32F429I-DISC1 carries. Family fixture `test/family_stm32f4/ltdc.cpp`
plus four negatives under `brio check stm32f4`.

## What the silicon does

**It is the one peripheral of this family whose output is a picture.**
The controller reads a rectangle of memory and drives a parallel RGB
panel: up to eight bits per channel plus HSYNC, VSYNC, DE and a pixel
clock, up to 1024x768, from two layers with eight input pixel formats,
a colour table per layer, per-pixel or constant alpha blending, colour
keying and a pseudo-random dithering stage. Nothing of it exists on
the parts without a display interface, and the reserve's
`ltdc_present()` is what a program asks; the accelerator beside it
(`stm32f4/dma2d.hpp`) is a WIDER fact, present on two parts of the
same class that have no panel interface at all.

**The pixel clock is a PLL of its own.** LCD_CLK comes off PLLSAI's R
output through RCC_DCKCFGR's PLLSAIDIVR (16.3.2, 6.3.24): a third PLL
beside the main one and the audio one, on the same root
(PLLCFGR.PLLSRC) and - this is the constraint that shapes the
arithmetic - **with no input divider of its own**. PLLSAI divides the
MAIN PLL's PLLM, so the VCO input is whatever the system clock already
fixed, and a pixel clock is an exact triple (N, R, the LCD divider) or
it is nothing. `lcd_clock_config_for()` solves that triple at compile
time and answers an empty configuration where no exact one exists,
which is a compile error at the call site rather than a panel running a
few per cent off.

**Three clock domains, and the bus stalls in all three.** Table 88 puts
the layer's address and length registers on HCLK, the reload and
interrupt registers on PCLK2, and everything else - the timings, the
global control, the background, the position counter and most of each
layer - on LCD_CLK. An access to a register of the pixel-clock domain
stalls the APB for six or seven PCLK2 periods PLUS five LCD_CLK ones,
which at a slow pixel clock is most of the cost (measured below: 216
core cycles a read at 6 MHz). And the domains are visible from the bus
in a way the chapter does not say: **with the block clocked by the APB
alone and no pixel clock at all, every register of the pixel-clock
domain reads ZERO whatever its reset value is** (measured), while the
PCLK2 and HCLK ones read theirs. The read answers rather than hanging
on a stall that cannot end.

**Every layer register but the colour table is shadowed** (16.4.1),
`LTDC_LxCR` included: a write goes into a shadow that the active
register takes either at once (SRCR.IMR) or at the beginning of the
first line after the active display area (SRCR.VBR), and until then a
READ RETURNS THE ACTIVE VALUE AND NOT WHAT WAS WRITTEN. Both reload
bits are set by software and cleared by HARDWARE, and neither can be
withdrawn. That is what makes a whole new picture - two layers, their
windows, their formats and their buffers - install on one frame
boundary; it is also why a read-modify-write of a layer register with a
reload pending loses the pending write, and why `configure()` writes
each register whole.

**And a read issued straight after a reload store races the reload**
(measured, below). The reload crosses from the store's own domain into
the pixel-clock one, and a read that starts before the crossing lands
still answers the pre-reload value; a read of ANY register of that
domain in front of it covers the crossing, since one such access is
five LCD_CLK periods of stall by 16.3.2. `reload()` and `wait_reload()`
spend that access themselves, with one read of GCR, so that a caller's
own read back is the reloaded value.

**A flag exists only while its enable is set** (measured, below). 16.7.9
describes LTDC_ISR as four status bits and says nothing about IER, but
a FIFO underrun that really happened leaves ISR clear when FUIE is
clear - the same rule this family's EXTI has for its pending bit
([exti.md](exti.md)). So a program that WATCHES a status instead of
taking an interrupt still has to set the enable; only the vector is
optional.

**Four events over TWO vectors** (16.5, figure 85): the line and the
register reload come out of the global one, the FIFO underrun and the
transfer error out of the error one. The four flags share ONE register,
so an ISR body that cleared everything standing would clear the other
vector's event - and would clear an underrun out from under a program
watching the flag. `Ltdc::isr()` takes a mask of what this vector
handles, and `ltdc_global_events` / `ltdc_error_events` are the two.

**The timings are stated accumulated and one short.** 16.4.1: SSCR
holds the synchronisation widths minus one, BPCR the synchronisation
plus the back porch minus one, AWCR that plus the active area minus
one, and TWCR that plus the front porch minus one. The driver takes a
panel's own eight numbers and does the accumulation. **16.4.1's worked
example does not obey 16.4.1's own rule for the two TOTAL fields**: its
SSCR, BPCR and AWCR words carry the minus one and its TWCR words
(0x294 and 0x1E7) are the plain sums. The driver follows the rule, and
the frame period measured against SysTick is what says the rule is
right.

**A layer's window is counted from the back porch.** 16.7.15 and
16.7.16: the first visible pixel of a line is AHBP + 1 and the first
visible line is AVBP + 1, so a layer's position registers hold an
accumulated number and not a picture coordinate. `LtdcLayer::window()`
reads the block's own back porches and does that arithmetic both ways.

**The frame buffer is described three times over.** LxCFBAR is the
address of the top left pixel, LxCFBLR holds the line length **in bytes
plus three** (16.7.23's own formula) beside the pitch in bytes, and
LxCFBLNR the number of lines. Together they say how much is fetched per
frame: fewer bytes than the window needs raises the FIFO underrun,
more is fetched and discarded.

**Blending is always on, even for a disabled layer** (16.4.2). Layer 1
is blended with the background, then layer 2 with that result; a
disabled layer contributes its DEFAULT COLOUR, which is why both the
default colour (transparent black) and the blending factors are left at
their reset value on a layer nobody is using. Only two of the eight
codes are legal in each of the two factor fields, and the pair the
reset value names is "pixel alpha times constant alpha" against "one
minus it".

**Dithering is two bits a channel and the width is READ-ONLY.** GCR's
DRW, DGW and DBW return what the block does (16.7.5) and cannot be set;
DEN is the switch, and it may be thrown while the controller runs.

**What the pads are is not the driver's.** Twenty-eight signals at
most, and how many bits of each channel a board wires is a schematic:
a panel narrower than 24 bits takes the MOST SIGNIFICANT bits of each
channel (16.3.3), so an 18-bit panel is wired to R[7:2], G[7:2] and
B[7:2] and the controller is still programmed in 24 bits. **The layer's
pixel format says what is in MEMORY, never what is on the wire.**

**What is NOT this driver's at all** is the panel's own controller. A
TFT module usually carries one that has to be told to take an RGB
interface before a pixel is fetched, and that conversation happens over
some other bus. It is the board's business and the application's.

## Types and verbs

**The pixel packers** - `argb8888`, `rgb888`, `rgb565`, `argb1555`,
`argb4444`: plain arithmetic, no register in sight, compiled on every
part of the family. A pixel is a number and which number it is depends
only on the format.

**`LtdcTiming`** - a panel's timings AS ITS DATA SHEET STATES THEM:
`hsync`, `hbp`, `width`, `hfp` and the vertical four, none of them
accumulated and none of them off by one, plus the four polarities
(`hsync_polarity`, `vsync_polarity`, `de_polarity` and `clock_edge`).
The arithmetic around it is free constexpr functions: `ltdc_total_width`
/ `ltdc_total_height`, `ltdc_frame_pixels`, `ltdc_frame_rate_mhz` (in
millihertz, because this framework has no floats),
`ltdc_fetch_bytes_per_second` (what a layer of a given format costs the
memory) and `ltdc_timing_valid`.

**`LtdcPixelFormat`** - the eight codes of 16.7.18, with
`ltdc_bytes_per_pixel`, `ltdc_format_indexed` and
`ltdc_al44_clut_address` (the 4-bit index replicated to eight bits,
which is where an AL44 table's entries really go).

**`LtdcWindow` and `LtdcLayerConfig`** - a rectangle inside the active
area, and everything one layer needs: the window, the format, the frame
buffer's address and pitch, the constant alpha, the two blending
factors, the default colour, the colour key and the CLUT enable. With
`ltdc_window_valid`, `ltdc_line_length` (the chapter's "+ 3") and
`ltdc_layer_config_valid`, which is what refuses a window off the edge,
a pitch narrower than its line, a length past its field and a colour
table on a format that has no index.

**`LtdcBlend1` / `LtdcBlend2` and `ltdc_blend_channel`** - the two
legal codes of each factor field, spelled so that no illegal one can be
written, and 16.7.21's formula in the integer arithmetic the silicon
does. The chapter's own worked example is one of the fixture's
static_asserts.

**`LtdcFramebuffer<Pixel>`** - a rectangle of memory as a typed
surface: `base`, `width`, `height`, an optional `pitch`, and
`line`/`write`/`fill`/`fill_rect` with clipping. The pixel type IS the
beat, which on an external memory is the difference between a fast
surface and a slow one. EVERY ACCESS IS A WRITE: the readers of a frame
buffer are the display controller and the accelerator, and ES0206 2.3.5
says the CPU is the one master that may not read FMC-held data safely
with interrupts live.

**`Ltdc`** - the block. `clock` (the APB2 gate every verb opens) and
`reset` (which puts all three clock domains back); `pixel_clock` in
three faces - solve and apply a rate, apply a triple solved at compile
time, or report what the registers make; `timing` both ways (the
panel's own numbers in, the same numbers out) and `horizontal_back_porch`
/ `vertical_back_porch`, which is what a layer's position is counted
from; `enable` / `disable` / `enabled`; `background`; `dither` and the
three read-only widths; `reload` / `reload_pending` / `wait_reload`;
`line_interrupt`, `interrupt`, `flag`, `flags`, `clear` and `isr` (the
body, taking the mask of what this vector handles); `position` (CPSR)
and `display_status` (CDSR). `irq_line` and `error_irq_line` are the
two vectors, `layers` is two.

**`LtdcLayer<1|2>`** - one layer, layer 1 at the bottom. `window` both
ways; `pixel_format`; `framebuffer`, `buffer_line` (length and pitch)
and `buffer_lines`; `constant_alpha`, `blending`, `default_colour`,
`colour_key` and `colour_keying`; `clut_entry` and `clut`; `enable` /
`disable` / `enabled`; and `configure`, which writes the whole layer
from one description in 16.6's order, WITH THE LAYER LEFT DISABLED and
no reload asked for - the caller enables and reloads when the whole
picture is ready, which is what makes a two-layer change atomic. It
takes the timing as an argument because a window's legality is a
question about the active area.

**On `Rcc` (`stm32f4/clock.hpp`)**: `pllsai_enable` / `pllsai_ready` /
`pllsai_wait` / `pllsai_configure` / `pllsai_config` and
`lcd_clock_divider` both ways, with `PllSaiConfig`, `LcdClockDivider`,
`pllsai_r_hz`, `lcd_clock_hz`, `LcdClockConfig` and
`lcd_clock_config_for` beside them - the verbs sit with the block that
owns the registers, as the audio PLL's do.

## How to use it

A panel brought up whole - the memory first, because the frame buffer
lives in it, then the pixel clock, then the pads, then the controller:

```cpp
using Panel = brio::Ltdc;
constexpr brio::LtdcTiming timing{.hsync = 10, .hbp = 20, .width = 240, .hfp = 10,
                                  .vsync = 2, .vbp = 2, .height = 320, .vfp = 4};
constexpr auto pll = brio::lcd_clock_config_for(8'000'000u, 6'000'000u, SysClock::pll.m);
static_assert(pll.pll.n != 0, "this root cannot make that pixel clock exactly");

board_claim_ltdc_pads();                        // AF14, and AF9 on four of them
(void)Panel::pixel_clock(pll);
(void)Panel::timing(timing);
Panel::background(0, 0, 0);
```

One layer over a frame buffer, enabled and reloaded together:

```cpp
const brio::LtdcFramebuffer<uint16_t> fb{Sdram::at<uint16_t>(0), 240, 320, 0};
fb.fill(brio::rgb565(0, 0, 0));

brio::LtdcLayerConfig layer{};
layer.window = {0, 0, 240, 320};
layer.format = brio::LtdcPixelFormat::rgb565;
layer.framebuffer = fb.address();
if (brio::LtdcLayer<1>::configure(layer, timing)) {
    brio::LtdcLayer<1>::enable();
    Panel::reload(brio::LtdcReload::immediate);
    Panel::enable();
}
```

A new picture installed on a frame boundary, which is what the shadow
registers are for - every write first, one reload last:

```cpp
brio::LtdcLayer<1>::framebuffer(other_buffer);
brio::LtdcLayer<2>::constant_alpha(200);
brio::LtdcLayer<2>::enable();
Panel::reload(brio::LtdcReload::vertical_blanking);   // both layers, one frame
```

The line event at the start of the vertical blanking - the moment a
program has a whole blanking in front of it:

```cpp
(void)Panel::line_interrupt(timing.vsync + timing.vbp + timing.height);
Panel::interrupt(brio::LtdcEvent::line, true);
Panel::enable_interrupt();

extern "C" void LTDC_IRQHandler() {
    const uint32_t seen = brio::Ltdc::isr(brio::ltdc_global_events);
    if ((seen & brio::ltdc_event_mask(brio::LtdcEvent::line)) != 0u) { ++frames; }
}
```

Watching the FIFO underrun without taking its interrupt - the enable is
still needed, only the vector is not:

```cpp
Panel::interrupt(brio::LtdcEvent::fifo_underrun, true);   // no Nvic::enable
Panel::clear(brio::ltdc_error_events);
...
if (Panel::flag(brio::LtdcEvent::fifo_underrun)) { /* the fetch did not keep up */ }
```

An indexed layer, its table loaded before the layer is enabled because
the table is the one thing no reload carries:

```cpp
for (uint32_t i = 0; i < 256; ++i) {
    brio::LtdcLayer<1>::clut_entry(i, palette[i].r, palette[i].g, palette[i].b);
}
layer.format = brio::LtdcPixelFormat::l8;
layer.clut = true;
```

## Bench findings

`test_stm32f4_ltdc` on the STM32F429I-DISC1, whose 240x320 panel is
driven in its RGB mode over eighteen data lines plus HSYNC, VSYNC, DE
and the pixel clock, with the frame buffer in the 64-Mbit SDRAM the
same board carries. The core at 180 MHz, the memory clock at 90 MHz and
LCD_CLK at 6 MHz. Every number below is that board's.

**The pixel clock, exactly.** From the 8 MHz crystal over the main
PLL's M of 4 - a 2 MHz VCO input - the triple is N 60, R 5 and the LCD
divider 4: a 120 MHz VCO, 24 MHz on the R output and **6.000 MHz** of
LCD_CLK. The panel's timings make a frame of 280 x 328 = **91840 pixel
clocks**, so 65.331 Hz and 15306 us. Measured against SysTick: 128
frames in 1959 ms, **15304 us a frame**, which implies a pixel clock of
6001 kHz - **0 per mille** off the arithmetic.

**THE THREE CLOCK DOMAINS ARE VISIBLE FROM THE BUS.** With the APB2
gate open and PLLSAI still off, GCR, CDSR, LxCACR and LxBFCR all read
**zero** - not their reset values - while SRCR, IER, ISR, LxCR and the
layer's three frame-buffer registers read theirs. With the pixel clock
running the same reads answer GCR 0x0000 2220 (the controller
disabled, every polarity active low, the three dither widths reading
two bits each), CDSR 0x0000 000F (every display signal reads ACTIVE
with the generator held), LxCACR 0xFF and LxBFCR 0x0607. The reads with
no pixel clock answer at once and do not hang.

**What the blocks hold at reset**, with the pixel clock up and nothing
written: both gates CLOSED and PLLSAI off; the four timing registers,
the reload, the background, both interrupt registers and the line
position all zero; both layers disabled, fully opaque and blending
0x0607.

**The timing registers.** The panel's 10/20/240/10 and 2/2/320/4 become
SSCR 0x0009 0001, BPCR 0x001D 0003, AWCR 0x010D 0143 and TWCR 0x0117
0147, and the driver reads the panel's own eight numbers back out of
them. 16.4.1's worked example gives SSCR 0x0007 0003, BPCR 0x000E 0005
and AWCR 0x028E 01E5 - exactly the words the chapter prints - and TWCR
**0x0293 01E6**, where the chapter prints 0x294 and 0x1E7: **the
example's two TOTAL fields are the plain sums and its own rule
subtracts one**, as its own AAW and AAH do. The measured frame period
above is what settles it: at the example's arithmetic a frame would be
15408 us and SysTick counted 15304.

**The four polarity bits are GCR's top four**, and nothing else of the
register moves with them: 0xF000 2220 with all four high, 0x0000 2220
back.

**A read of a pixel-clock-domain register costs 216 core cycles** (1.2
us at 180 MHz): sixteen reads of CPSR took 3463 cycles. Five LCD_CLK
periods at 6 MHz is 833 ns on its own, so 16.3.2's stall is nearly all
of it and a polling loop on that register pays it every turn.

**The position counter walks the whole frame.** Over 200 ms it covered
x 0..279 and y 0..327 against a frame of 280 x 328 - porches and
synchronisation included - and never left it. All four bits of CDSR
were seen in both states over one frame: the register is the live state
of the four signals and not a latch.

**The line event lands where it was asked for, once a frame.** 64
events in 980 ms, one every **15312 us** against the 15306 the timing
predicts. The handler read the position counter at line 324 for an
event programmed at line 324 of 328. A vertical-blanking reload raises
its own event on the same vector.

**The shadow registers, measured.** CACR written 0x80 reads 0xFF before
the reload and 0x80 after - 16.4.1's own sentence on the silicon. The
immediate reload's bit is already clear on the next read: the hardware
took it inside the store's own bus cycle. A vertical-blanking reload
stands pending and is taken at the next blanking - 7 ms of the 15 a
frame lasts, in one run, which is only where in the frame the store
landed - and never later than that.

**AND A READ STRAIGHT AFTER A RELOAD STORE RACES THE RELOAD.** Eight
rounds each way, a window written and reloaded and the register read
back at once: through a RAW store into SRCR the first read answered the
PRE-RELOAD value **eight times out of eight**; through the driver's own
`reload()`, which reads GCR after the store, it answered the reloaded
value **eight times out of eight**. One access of the pixel-clock
domain is five LCD_CLK periods of stall by 16.3.2 and the crossing is
shorter than that, which is why the verb closes the race rather than
narrowing it. The trap is in the silicon and not in the surface above
it.

**A layer's geometry, decoded.** With AHBP 29 and AVBP 3, a window at
40,60 of 100x120 becomes WHPCR 0x00A9 0046 and WVPCR 0x00B7 0040 - the
back porch plus one plus the offset, and a stop that is the LAST
visible pixel and not one past it - and comes back as 40,60 of 100x120.
A 100-pixel line of 16-bit pixels at a 480-byte pitch is CFBLR
0x01E0 00CB: 203 = 200 + 3 beside 480. The constant alpha, both
blending factors, the default colour and the colour key are all written
and read back (0x30, 0x0405, 0x1122 3344, 0x00AA BBCC). **The layer's
ENABLE is shadowed too**: it reads clear until the reload takes it,
which is what lets a whole new picture install on one frame boundary.

**One layer of 16-bit pixels never starves the FIFO**: 10034 kB a
second fetched out of the external memory, and the underrun flag clear
over a hundred frames, with no transfer error.

**THE FLAG EXISTS ONLY WHILE ITS ENABLE IS SET.** A frame buffer
described with half the lines it needs, or with half the bytes a line,
raises the FIFO underrun with FUIE set - and leaves ISR CLEAR with FUIE
clear, over the same starved fetch. A program that polls a status
register it never armed polls a register that cannot answer.

**AND THE UNDERRUN IS A STORM, NOT AN EVENT.** With the error vector
armed over that same starved layer, the handler was entered **33194
times in three frames** - one every 1.4 us, which is per PIXEL the FIFO
could not serve and not once a line, let alone once a frame. The error
vector is a thing to arm over a picture that is known good; the FLAG is
the thing to watch over one that is not.

**THE BANDWIDTH THE DISPLAY TIER LIVES ON.** The accelerator filling
the same memory as fast as it can, against three display
configurations, with the underrun flag armed throughout:

| displayed | fetched | fill | fill rate | underrun | total over the bus |
|-----------|---------|------|-----------|----------|--------------------|
| one layer, 16 bpp | 10.0 MB/s | 39..40 px/us | 156..160 MB/s | none | 166..170 MB/s |
| one layer, 32 bpp | 20.1 MB/s | 36..37 px/us | 144..148 MB/s | none | 164..168 MB/s |
| two layers, 32 bpp | 40.1 MB/s | 30 px/us | 120 MB/s | none | 160 MB/s |

The total is the same in all three within six per cent, and the
display's fetch comes off the top of it: **this memory delivers some
160 MB/s and the accelerator gets what the panel has not taken**. No
configuration starved the controller - two layers of 32-bit pixels
blended at 65 Hz, with a second master writing the same device, is
still inside what this board's SDRAM can do. The ranges are the spread
over several runs, a pixel a microsecond, which is the refresh landing
differently against the fill. What each figure costs is the device's
rate, the bus width and the refresh, all three of them this board's and
not the family's. The accelerator's AHB dead time is the one knob: 32
cycles between accesses drop the fill from 30 to 25 pixels a
microsecond.

**An indexed layer works and its table cannot be read back.** An L8
layer of one byte a pixel, with 256 entries written into the
controller's own colour table, is fetched over thirty frames with no
underrun; LxCLUTWR is write-only and no reload carries it, so what it
PAINTS is the one thing a register cannot say.

**The panel answers, and that is how the RGB mode is confirmed.** Its
own controller has no read path on this board (the SPI chapter measured
that: the board straps the interface so that replies leave on a pad the
MCU is not wired to), so the initialization sequence - 106 bytes over
SPI5 at 2812 kHz, ST's own for this panel - can only be measured by
what it makes the device do. Command 0x35 turns the panel's
tearing-effect output on, and the pad then saw **66 edges in 500 ms**
against 32.7 frames: 33 pulses, one a frame. The panel took the
sequence AND its frame timing is locked to the controller's.

**And a person can look at it.** The suite's last letter paints eight
colour bars with the accelerator and steps a block along the bottom
once a frame: 196 steps over 196 frames in three seconds, with the
underrun clear throughout.

## Not covered yet

Driver gaps:

- **The panel's own controller** - declined, and not a gap in this
  chapter: a TFT module's controller is reached over some other bus and
  what it wants is the module's, not the display controller's. The
  bench suite carries the sequence for the one panel this desk has,
  in its own source, because a measurement needs a live panel.
- **LCD_CLK is not rebased by a clock change** - the driver is
  deliberately not a `ClockUser`, for the same reason
  [fmc.md](fmc.md)'s is not: PLLSAI divides the MAIN PLL's M, so a
  dynamic clock that changes M changes the pixel clock and every
  timing under it, and re-solving the triple means stopping the panel.
  A program that rescales its core clock states the pixel clock once,
  at the rate it has chosen to stay at.
- **A linker region over the external memory** - no `.sdram` output
  section and no crt hook, so a frame buffer is reached through a
  pointer and not through named objects. The three things placing data
  there needs are listed in [fmc.md](fmc.md)'s own gap list.

Implemented but not bench-verified:

- **What the picture actually looks like, pixel by pixel** - the
  window's position on the screen, the colour key, the default colour
  outside a window, the constant alpha of a blended pair, the dithering
  on and off, and the colour table's own colours. Every one of them is
  written and read back, and every one of them needs EYES or a camera:
  the controller has no read path into what it painted, and this board
  has no read path into the panel. A frame grabber on the eighteen data
  lines, or a photograph, is what would measure them.
- **The transfer error (TERRIF) and its vector** - the flag, TERRIE and
  the ISR body are all in the driver; nothing the suite can safely do
  provokes an AHB error on a fetch. A layer pointed at an address the
  bus matrix faults on would raise it.
- **The other five pixel formats** - RGB888, ARGB1555, ARGB4444, AL44
  and AL88 compile, are refused where the chapter refuses them, and are
  never shown: RGB565, ARGB8888 and L8 are what the suite displays. A
  format is a memory layout, so what would measure the rest is the same
  eyes as above.
- **Pixel clocks other than 6 MHz** - one panel, one rate. The
  arithmetic is exact and compile-time checked over the whole
  (N, R, divider) space, and DS10693 table 107's ceiling of 83 MHz is
  stated and not enforced: what a second panel would measure is whether
  a fast pixel clock changes the register stall of 16.3.2, which at 6
  MHz is nearly all of a read's cost.
- **The F469/F479's LTDC on its own pads** - on the 32F469IDISCOVERY
  the controller's output goes to the DSI host and not to a pad, and
  [dsi.md](dsi.md)'s suite is where it drives pixels (a frame of 16- or
  32-bit pixels out of that board's SDRAM per refresh of the host's
  adapted command mode, the pixel clock at 26.4 MHz, 61 frames a
  second); this suite stays the DISC1's, and the RGB pads of that class
  would need a board that wires them.
