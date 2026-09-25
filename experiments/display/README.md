# display - an ILI9481 panel and an XPT2046 touch controller on one SPI bus

A breadboard experiment, ASSEMBLED ONCE AND DISMANTLED. Nothing of it
is wired on any bench today. The four apps are kept because they are
brio's first arbitrated-bus application (two devices, one `SpiBus`,
per-request clock and mode) and because the panel is a candidate for
the first portable example when it is cabled again. This directory is
self-contained and nothing under docs/ references it.

## What it was

The 3.5" red module **HST035003-A**: controller **ILI9481** (320x480,
SPI = 18-bit pixels only, the panel needs INVON), an **XPT2046**
resistive touch controller on board (U2; U1 is the LDO), backlight tied
high. An older 2.4" module that stayed white through every protocol
despite verified signals is parked as defective-suspect.

- `display_id` reads the controller's DCS registers (RDDPM, RDDID, ID4,
  the 0xBF device code) and prints the raw answers - the probe that
  identified the controller.
- `display_fill` cycles a full-screen solid fill red/green/blue (CASET/
  PASET + RAMWR/3C row writes, INVON).
- `spi_duo` is the point: ILI9481 fills (960-byte rows at 6 MHz) and
  XPT2046 touch polling (3-byte conversions at 1.5 MHz) through the
  same `SpiBus`, the touch steering the fill palette, the clock switched
  per request.
- `spi_paint` paints on the panel with the pen through the same bus.

## The wiring it needed (AVR128DB48, rail at 3.3 V)

| Signal | Pin |
|--------|-----|
| SPI0 MOSI / MISO / SCK (DEFAULT route) | PA4 / PA5 / PA6 |
| Display CS / RS(DC) / RST | PD0 / PD1 / PD2 |
| SD_CS (module, unused) | PD4 |
| T_CS (touch) | PD5, PEN unused |

The module has no level shifter, so the desk had to run at 3.3 V; the
same DEFAULT route at 5 V would have destroyed it, which is why the
bench suites use SPI0 ALT1 (PE0..PE3) instead.

## Building

Discovered by the avrdx project like any other app
(`experiments/*/avrdx/*.cpp`): `cmake --build --preset
avr128db48-release --target spi_duo` (or the other three).

## The STM32F4 half: the panel's GRAM read back as an oracle

`experiments/display/stm32f4/ili9481_probe.cpp` is the same module on a WeAct STM32F411CE
black pill, and its point is different from the AVR apps': not whether
the panel shows colours but whether the program can READ WHAT IT WROTE.
A frame memory that answers is the bench plane of design/gfx.md's three
planes of truth - a display driver and every primitive above it can be
judged pixel for pixel against what the silicon holds, with no eye on
the glass. The app is a letter menu over the board's own USB-C (CDC
ACM, `brio run OU <letter>`): the controller's read registers, the read
stream's alignment found by search, read_memory_continue, the whole
frame written and read back and timed, the write and read rate ladders
judged by read-back, the eight MADCTL scan orders, and brio/gfx drawing
through a `Surface` over the panel with one word read back against the
font. The bus is driven synchronously through `SpiHost::start()`
(polled requests, the arbiter skipped: a device that owns its bus alone
may).

Predictions, written before the first run: the device code reads back
`02 04 94 81` after ONE DUMMY BYTE (the datasheet lists a dummy read as
the first parameter of every read command and the serial figures show
no dummy clock); RAMRD returns three bytes a pixel with the six MSBs of
each holding the colour and the two LSBs zero; a 12 MHz write (the
datasheet's 12.5 MHz ceiling) reads back exact and 24 MHz does not; a
6 MHz read (above the 4.17 MHz ceiling) is where the read path starts
to fail; the whole frame (460800 bytes) writes in about 0.8 s at 6 MHz
through the polled pump and reads back in about 1.5 s at 3 MHz.

### What it found (35 verdicts, all passing; `brio run OU z`)

- **The controller answers on SDO.** The device code (BFh) reads
  `02 04 94 81` behind ONE DUMMY CLOCK - the stream is a bit late, so
  the datasheet's "dummy read" is a clock and not a byte for that
  command. The single-parameter registers (RDDPM 0x1C, RDDMADCTL 0x00,
  RDDCOLMOD 0x66 = 18 bpp) arrive byte-aligned with no dummy at all;
  RDDID (04h) and RDDST (09h) read all ones on this module. RDDPM's D7
  reads 0 as the bit table says ("set to 0"), whatever the prose under
  it says about the booster.
- **The frame memory read (RAMRD, 2Eh) is ONE DUMMY BYTE (it reads
  0x80) and then three bytes a pixel**, the six MSBs of each the colour
  and the two LSBs zero - a different alignment from the device code's.
  read_memory_continue (3Eh) has its own dummy byte and picks up at the
  pixel after the last one clocked; a pixel cut short by spare clocks
  is read again from its first byte. Clock exactly what you need.
- **The whole frame (460800 bytes) reads back byte for byte** as
  written: through the polled pump 1070 ms to write at 6 MHz (430 kB/s)
  and 1845 ms to read at 3 MHz (249 kB/s); through SPI1's two DMA
  engines 693 ms and 1499 ms at the same rates (664 and 307 kB/s, the
  wire being 750 and 375), and 386 ms to write plus 573 ms to read at
  12 MHz both ways (1193 and 804 kB/s), still exact.
- **Rates.** A write at 12 MHz (the datasheet's 12.5 MHz ceiling) reads
  back exact; 24 and 48 MHz leave garbage. A READ is exact at 6 and at
  12 MHz - three times the datasheet's 4.17 MHz - and wrong at 24 MHz.
  One module, one desk: printed, and the defaults stay inside the
  datasheet - and on another day, the analyser's probes gone from the
  lines, the same two 12 MHz verdicts (letters e and k) failed while
  everything at 6 MHz passed: at that rate the breadboard decides, which
  is the pad-slew finding of docs/stm32f4/spi.md seen from the other
  side - and with the data pad slowed to `medium` through the host's
  `mosi_speed()`, SCK left at `very_high`, all 38 pass again, the 12
  MHz frame through the engines included.
- **MADCTL's B5/B6/B7 change the order the address counter walks the
  WINDOW, not the coordinate system**: under 40 (columns reversed)
  logical (0,0) of a window (0..3, 0..1) is physical (3,0), not (319,0);
  under 80 it is (0,1); under 20 columns and pages exchange, CASET then
  takes the PAGES (0..479, a full column reaches page 479 with or
  without B6) and PASET the columns (a PASET past 319 collapses to one
  pixel). The write and the read agree under all eight codes, so a
  read-back under the same MADCTL is an oracle in every orientation.
  **The BGR bit (08) acts on writes only**: a pixel written under it is
  stored with R and B exchanged and read back as stored.
- **Two facts of this module, seen on the glass and not in the GRAM:**
  it is wired BGR (the driver writes B, G, R and the read-back stays in
  the driver's own order), and the glass shows MEMORY COLUMN 319 AT ITS
  LEFT - a picture drawn under MADCTL 00 is mirrored. The rotation map
  folds that mirror in: a rotation is the usual one in DISPLAY
  coordinates, the memory column is 319 - d, and each rotation states
  the walk bit along its runs (r0 = B6, r90 = B5, r180 = none, r270 =
  B5 + B7); the bits ACROSS a run stay clear because the two Surface
  verbs never send a multi-row block that is not uniform. Letter h
  reads "brio" back through the same map under all four rotations,
  byte-exact, and ALL FOUR ROTATIONS WERE JUDGED BY EYE against the
  panel's own marking (the probe's letter o: a red square at the
  logical origin, an arrow at the top, corner labels): r0 is the
  panel's portrait, r90 the picture turned clockwise, r180 upside
  down, r270 counter-clockwise, the text readable in each. The
  display-side bits of MADCTL, changed with the memory untouched
  (letter n): B1 (horizontal flip) and B4 (line address order) DO
  NOTHING on this module; B0 (vertical flip) mirrors the glass top to
  bottom - a mirror, so the text stays unreadable turned around. The
  module's own horizontal mirror can therefore not be undone by B1 and
  stays in the map.
- **The overflow that cost an afternoon**: the row buffers were sized
  by the physical row (320 pixels) while a run of the rotated surface
  is 480 wide, and a clear() under r90 wrote 476 bytes past
  `row_buf` - straight over the rotation and the CDC console's own
  state (`configured_`, DTR, the counters). The board then spun in
  print() with a console that thought itself unconfigured, the host's
  open() blocked, and the half-drawn picture looked like a panel limit.
  Diagnosed with OpenOCD on the halted board and `nm` for the
  addresses; the buffers are sized by the panel's long side now.

- **brio/gfx draws on it through a Surface of two verbs** (a window and
  a row per fill_rect row, a window and a row per write_run): the word
  "brio" in Font5x7 reads back with zero bytes differing from the font's
  own rows, and a filled rectangle's pixel reads back its colour. A
  clear costs 1002 ms and the whole test picture 1504 ms on the pump.

### The memory's rules at the edges (letters w, l, s; not in the ALL run)

The host simulator (brio/host/sim_dcs_panel.hpp) had six rules the
letters above never measured, each marked as an assumption. Three
letters measure them in MEMORY coordinates under MADCTL 0x00, with
pixels that carry their own index (byte 0 = (i + 1) << 2, byte 1 = 0x40,
byte 2 = (63 - i) << 2), so a read-back says WHICH pixel landed WHERE;
each verdict passes when the outcome is one the letter can name, and
the finding is the text beside it. What they found, and what the
simulator now does:

- **The counter WRAPS at the window's end.** Twelve pixels into a window
  of eight (columns 104..107, pages 202..203): the ninth lands on the
  window's first cell and the first four cells hold tags 8..11. A read
  of twelve from the same window answers the first four pixels again
  after the eighth. Neither a write nor a read past the window's last
  pixel is dropped or answered with a constant.
- **The address counter is an ADDRESS, and only RAMWR and RAMRD move
  it.** The window re-issued after two pixels - CASET and PASET, or
  CASET alone, or PASET alone - and then 3Ch: the pixel lands on the
  THIRD cell, nothing rewound. A window one column to the right after
  two pixels: 3Ch lands at column 106 - the same address, not the same
  index. A window far away (columns 220..223, pages 104..105): three
  pixels through 3Ch land NOWHERE. The two counters are independent: a
  RAMRD of two, the window re-issued and a 3Eh of one answers pixel 2;
  a RAMRD of two, a RAMWR of one and a 3Eh of one still answers pixel
  2; a RAMWR of one, a RAMRD of three and a 3Ch of one lands on the
  second cell.
- **A pixel cut short at the close of a write is DROPPED.** A RAMWR of
  four bytes into a window of three (one pixel and one byte) and a 3Ch
  of five (the other two bytes and a whole pixel): the second cell
  holds the 3Ch's first three bytes as a pixel and the third cell is
  untouched. The continuation starts a fresh pixel; the byte left over
  is gone and the address did not move for it.
- **Windows the axes cannot hold are two cases, not one.** Each after a
  sentinel window (columns 8..11 of page 106) and with ONE pixel
  written, so that an ignored command is told from a dropped write and
  no wrap hides the pixel: CASET 400..403 - the pixel lands in the
  sentinel, the command IGNORED and the window before it standing; CASET
  316..323 (an end beyond the axis) - the pixel lands nowhere, the
  command TAKEN and the window invalid; CASET 210..205 (backwards) -
  nowhere, and a RAMRD of two from that window answers `A8 A8 A8 | A8 A8
  A8`, an odd page's reset fill, as if the address had left the array
  (the value is recorded, the reason is not known); CASET 304..307 then
  PASET 500..501 - the pixel at column 304 of page 106, the PASET
  ignored; CASET 304..307 then PASET 105..103 - nowhere.
- **MADCTL after the window reassigns the axes.** CASET 104..107 and
  PASET 202..203 written under 0x00, then MADCTL 0x20, then eight
  pixels: the first at column 202 of page 104, the second at column 202
  of page 105 - the registers keep their bytes, CASET's became the
  pages, and the walk steps the pages first. The one of the six
  assumptions that held.

A letter that writes more pixels than the window holds and looks for
the first of them finds nothing: the wrap hides every one, and
"dropped" is the wrong answer. One pixel, and a sentinel window before
the command under test, is the shape that measures this.

Two desk notes. The panel needs INVON, as on the AVR. And a freshly
enumerated CDC console can echo the boot banner back into the board as
commands when something opens the tty before `brio run` does (the
letters of "clk=PLL96" ran c and k once); harmless, and `brio run`'s
wake swallows what follows.

### Two devices on the bus: `experiments/display/stm32f4/spi_duo.cpp`

The same module's XPT2046 shares SCK, MOSI and MISO with the panel, on
its own chip select, and `spi_duo` puts both behind ONE `SpiBus` (the
BusMaster arbiter) with the two DMA engines of SPI1 carrying every data
phase at 6 MHz: a Painter that owns the glass and the F469 suite's game - a
white square at ten random places, every tap's raw coordinates
recorded against the square's centre, from the third tap the best of
the two axis assignments with a least-squares line per axis marks
where the tap was understood (a red cross), after the tenth the fit is
printed and the pen paints dots with it - every mark a rectangle fill
queued in a small FIFO, one row a request, and when the FIFO is empty a
band of changing colour sweeps the bottom strip so the bus is never
idle; and a Touch that polls every 20 ms at 1.5 MHz (a Z1 pressure
gate with the chip powered down between polls - PD = 00, the only
setting that leaves PENIRQ alive -, then X and Y as bursts of eight
conversions, trimmed means, a tap after three idle polls and two
pressed samples, never in the first half second). The arbiter is a
FIFO and the Painter posts one request at a time, so a touch waits at
most one row. Measured: about 500 rows a second painted at 6 MHz, 50
touch conversions a second interleaved, zero requests rejected. The
console (output only) prints every tap with its raw values, the two
RESULT lines of the fit, and a report every two seconds that includes
the square's own row read back through the bus. Wiring beyond the
panel's: T_CLK PA5, T_DIN PA7, T_DO PA6, T_CS PA4, PEN (T_IRQ) PA1.

The game's result on this module (ten taps, a terminal attached, the
two farthest taps trimmed from the final fit): the axes are SWAPPED -
the logical x of the landscape surface comes from the controller's Y
reading and the logical y from its X reading, inverted -

    x = raw_y * 0.1303 - 13      (raw_y 106 at x = 0, 3779 at x = 479)
    y = raw_x * -0.0952 + 343    (raw_x 3603 at y = 0, 255 at y = 319)

with 3 px rms and 5 px at worst over the ten, every tap on its square;
the first game had two taps 180 px off, both taken while the finger
was still landing, which is why a tap is now the fourth pressed sample
of a stroke and the fit drops its two worst residuals.

What building it found:

- **The touch controller's select must be HIGH in every program on
  this bus**: with PA4 left floating the XPT2046's DOUT talks over the
  panel's answers and every read is garbage (the probe reads 7F 7E FE
  FE for the device code). The probe drives PA4 high now.
- **At 12 MHz the breadboard's wires are the limit, not the driver**
  (docs/stm32f4/spi.md's finding): with SCK and MOSI both on
  `very_high` pads the panel took nothing spi_duo sent at PCLK2/8
  through the interrupt pump, whatever the data; slowing EITHER pad to
  `medium` lands every byte, and hanging a logic analyser's probes on
  the lines makes every case pass at every slew class - the probes'
  capacitance does what the pad setting does. The analyser on SCK alone
  leaves the case failing and shows a clean clock at the connector;
  on MOSI alone it makes it pass: MOSI's fast edge is the actor, the
  damage is beyond the connector. 6 MHz is exact at any class. OPEN: with
  the pad at medium the probe lands 12 MHz requests exact and spi_duo,
  under the kernel with the touch's requests interleaved, still did
  not; it runs at 6 MHz, which on a printed board would not be needed.
- **A reflash fills the GRAM with 0x54/0xA8 in alternate pages** (the
  MCU's reset leaves the panel's lines floating), so a census taken by
  the probe after flashing another program describes the reset, not
  the program: a program verifies its own writes in the same run,
  which is what spi_duo's report does.
- A spin-wait on a flag an interrupt sets must be `volatile`: a probe
  letter without it "measured" an interrupt path that never completed
  and corrupted every write - two false verdicts for one missing word.

### The wiring it needs (STM32F411CE black pill, rail at 3.3 V)

| Signal | Pin |
|--------|-----|
| SPI1 SCK / MISO (module SDO) / MOSI (module SDI), AF5 | PA5 / PA6 / PA7 |
| Display CS / RESET / DC-RS | PB2 / PB1 / PB0 |
| VCC and LED (the backlight) | 3.3 V |
| Touch controller (T_CLK, T_CS, T_DIN, T_DO, T_IRQ) | left open |
| Console | the board's USB-C (CDC ACM 1209:0001) |
| Probe | STLINK-V3 on the SWD header, no NRST |

PB2 (the chip select) is BOOT1 on this part: sampled only with BOOT0
high, a plain output afterwards. The board is powered from its USB-C; the STLINK-V3's target
pin senses the rail and does not feed it.

### Building

Discovered by the stm32f4 project like any other app
(`experiments/*/stm32f4/*.cpp`): `cmake --build --preset
stm32f411ce-release --target ili9481_probe`, or `brio flash O
ili9481_probe`.
