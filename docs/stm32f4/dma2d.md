# DMA2D, the Chrom-Art accelerator (STM32F4)

Documents of record: RM0090 Rev 22 ch. 11 - RM0390 Rev 6 and RM0383
Rev 4 have no such chapter, because the F446 and the F411 have no such
peripheral. Errata: ES0206 Rev 24's sixteen sections do not include
this one; the accelerator and the display controller beside it are the
only two chapters of this stratum with no erratum at all. Driver:
`stm32f4/dma2d.hpp`; the presence facts are in
`stm32f4/device_tables.hpp`. Bench suite: `test_stm32f4_ltdc`, which
measures the accelerator beside the display controller because they
share the memory that makes either of them interesting. Family fixture
`test/family_stm32f4/dma2d.cpp` plus two negatives under `brio check
stm32f4`.

## What the silicon does

**It is a DMA that knows what a pixel is.** Rectangles rather than
runs: it fills an area with one colour, copies one area into another,
converts between eleven input formats and five output ones, and blends
two sources into a third - all with a line offset at each end, so a
rectangle inside a wider image is an address and two offsets and not a
loop. It is a bus master of its own, which is the other half of why it
matters: reads it makes are not the CPU's, and on this family that is
the difference between a safe read of external memory and ES0206
2.3.5's unsafe one.

**Its presence is a WIDER fact than the display controller's.** Every
part with a panel interface has the accelerator; two parts of the same
class have the accelerator and no panel interface, because filling and
converting memory is useful with no display attached. `dma2d_present()`
and `ltdc_present()` are two questions and the reserve asks them
separately.

**The four modes are one engine with parts switched off** (11.3.11).
Register-to-memory writes one colour and fetches nothing.
Memory-to-memory fetches through the foreground FIFO as a plain buffer
with NO conversion - the format field says only how wide a pixel is,
and the output's format is not consulted at all, which is why the
driver refuses a pair whose widths differ rather than writing a
rectangle of nonsense. Memory-to-memory with the pixel format converter
runs the foreground through its converter. The blending mode fetches
both sources, converts each and mixes them.

**The offsets are in PIXELS and the addresses are BYTES.** FGOR, BGOR
and OOR count pixels of their own format (11.5.5), added at the end of
every line; FGMAR, BGMAR and OMAR are byte addresses that must be
aligned to the format's own width - four bytes for a 32-bit format, two
for a 16-bit one, one for the rest, which is 11.5.4's sentence with the
24-bit format (three bytes, no wider alignment than one) filled in.

**The silicon checks the configuration and answers with a FLAG, not a
refusal.** 11.3.11 lists a dozen ways to be wrong - a misaligned
address, a colour mode that is not one of the codes, an odd pixel count
or an odd line offset with a 4-bit format, a zero in either field of
NLR - and the engine answers all of them by raising CEIF when START is
written and transferring nothing. The driver checks what it can before
the store (`dma2d_source_valid`, `dma2d_output_valid`,
`dma2d_area_valid`) so that a caller gets a false instead of a flag;
what is left to the flag is what only the silicon knows.

**One transfer at a time, and a CLUT load is a transfer.** CR.START is
cleared by hardware at the end, on an abort, on an error and on a
configuration error; FGPFCCR.START and BGPFCCR.START are the same bit
for the two automatic colour-table loads, and 11.3.11 says a table load
cannot run beside a data transfer. An abort stops the transfer and does
NOT raise the transfer-complete flag; a suspend stops it where it is
and either resumes or is abandoned.

**The colour tables are two 256-word memories, filled two ways.** The
engine loads one out of memory by itself (the address in FGCMAR, the
size in FGPFCCR.CS as the count minus one, the format in CCM, then
START), or the CPU writes the words straight into FGCLUT/BGCLUT. The
CPU may not touch a table while the engine is using it: that raises
CAEIF.

**The blend is an equation with a stated rounding** (11.3.11): the
output alpha is the union of the two, each channel is the two weighted
by their alphas over that union, and "divisions are rounded to the
nearest lower integer". `dma2d_blend_alpha` and `dma2d_blend_channel`
are that equation in the integer arithmetic the silicon does, which is
what a test judges the block against.

**The alpha of each source can be kept, replaced or multiplied**
(11.5.8's AM field), and a source whose format carries no alpha gets
0xFF. The two alpha-only formats, A4 and A8, carry no colour at all:
the colour comes from FGCOLR or BGCOLR, which is what makes them a
one-bit-deep stencil painted in a fixed colour.

**The AHB dead time is the only bandwidth knob** (11.3.15): a
guaranteed minimum number of cycles between two of this master's
accesses, which is how a program stops it starving another master on
the same memory. It is a real throttle and the bench findings say by
how much.

## Types and verbs

**`Dma2dColor`** - the eleven input formats of table 53, whose first
eight are the display controller's own eight in the same order.
`Dma2dOutputColor` - the five output ones (there is no colour-table
generator, so an indexed output does not exist). With
`dma2d_bits_per_pixel` for both, `dma2d_format_indexed`,
`dma2d_format_alpha_only` and `dma2d_alignment`.

**`Dma2dSource`** - one input: the address, the line offset in pixels,
the format, the alpha mode and the ALPHA field, the colour an A4 or A8
source is painted in, and the colour table an indexed format needs
(address, entry count, format). **`Dma2dOutput`** - the address, the
line offset and one of the five formats. **`Dma2dArea`** - the
rectangle, `pixels` a line and `lines`.

**`Dma2dMode`, `Dma2dAlpha`, `Dma2dClutColor`, `Dma2dEvent`** - the
register's own codes, and `dma2d_event_mask` for the bit each event
holds in ISR, in IFCR and (eight higher) in CR's enables.

**The arithmetic, as free constexpr functions**: `dma2d_output_colour`
(what OCOLR must hold for a fill of a given format, from an ARGB8888
colour), `dma2d_source_alpha` (what a converter does to an alpha) and
`dma2d_blend_alpha` / `dma2d_blend_channel` (11.3.11's equation). And
the refusals: `dma2d_area_valid`, `dma2d_source_valid` and
`dma2d_output_valid`.

**`Dma2d`** - the block. `clock` and `reset`, `init` (the gate open and
every flag cleared); `busy` and `wait`, `abort` and `suspend`; the four
modes as four verbs - `fill`, `copy`, `convert` and `blend` - and
`start` for a shape they do not have; `load_clut`, `clut_busy`,
`wait_clut`, `clut_entry` (the CPU's own write) and `clut_word` (the
read back, deliberately not an overload of the setter); `interrupt`,
`flag`, `flags`, `clear` and `isr`; `watermark`; and `dead_time` both
ways. `irq_line` is the one vector.

## How to use it

A rectangle filled with one colour, the colour stated in ARGB8888 and
packed into the output's format by the driver:

```cpp
brio::Dma2d::init();
const brio::Dma2dOutput frame{.address = fb.address(), .line_offset = 0,
                              .format = brio::Dma2dOutputColor::rgb565};
(void)brio::Dma2d::fill(frame, brio::argb8888(255, 0, 0, 255), {.pixels = 240, .lines = 320});
(void)brio::Dma2d::wait();
```

A rectangle inside a wider surface - which is what the line offsets
are for, and the shape a "fill this box" verb really has:

```cpp
const brio::Dma2dOutput box{.address = fb.address() + 2u * (y * 240u + x),
                            .line_offset = static_cast<uint16_t>(240u - w),
                            .format = brio::Dma2dOutputColor::rgb565};
(void)brio::Dma2d::fill(box, colour, {.pixels = w, .lines = h});
```

A copy with a pixel format conversion - 16-bit pixels in, 32-bit out,
with the alpha the source has not:

```cpp
(void)brio::Dma2d::convert(
    {.address = source, .format = brio::Dma2dColor::rgb565},
    {.address = destination, .format = brio::Dma2dOutputColor::argb8888},
    {.pixels = 64, .lines = 32});
```

Two images blended into a third, the foreground on top:

```cpp
(void)brio::Dma2d::blend({.address = sprite, .format = brio::Dma2dColor::argb8888},
                         {.address = background, .format = brio::Dma2dColor::argb8888},
                         {.address = destination, .format = brio::Dma2dOutputColor::argb8888},
                         {.pixels = 64, .lines = 32});
```

An indexed source, its table loaded by the engine first - a table load
is a transfer and cannot run beside one:

```cpp
const brio::Dma2dSource indexed{.address = pixels, .format = brio::Dma2dColor::l8,
                                .clut_address = reinterpret_cast<uint32_t>(palette),
                                .clut_entries = 256};
if (brio::Dma2d::load_clut(indexed) && brio::Dma2d::wait_clut()) {
    (void)brio::Dma2d::convert(indexed, out, area);
}
```

Completion taken as an interrupt instead of waited for:

```cpp
brio::Dma2d::interrupt(brio::Dma2dEvent::transfer_complete, true);
brio::Dma2d::enable_interrupt();
extern "C" void DMA2D_IRQHandler() { const uint32_t seen = brio::Dma2d::isr(); ... }
```

Throttled, so that a display controller reading the same memory is not
starved:

```cpp
brio::Dma2d::dead_time(32, true);      // 32 AHB cycles between accesses
```

## Bench findings

`test_stm32f4_ltdc` on the STM32F429I-DISC1, the core at 180 MHz. The
byte-exact letters work in INTERNAL memory, because the CPU is the one
master that may not read the external one safely with interrupts live
(ES0206 2.3.5); the bandwidth letter works in the external one,
because that is where a frame buffer lives.

**What the block holds at reset**, with the gate opened and nothing
else touched: the AHB1 gate itself is CLOSED, CR = 0, ISR = 0,
OPFCCR = 0 and AMTCR = 0 - idle, no interrupt armed, no flag standing
and the dead time off.

**The four modes, byte for byte** over a 16x8 rectangle:

- **Register to memory** wrote 0x8012 3456 into all 128 words and
  nothing else.
- **Memory to memory** moved 128 words unchanged.
- **With the pixel format converter**, RGB565 into ARGB8888 is exact
  against 16.4.2's own rule - each channel expanded by REPLICATING its
  most significant bits into the least, and an opaque alpha filled in
  where the source has none.
- **Blending** 0x80C0 4020 over 0xFF10 80F0 gave **0xFF68 5F87**, which
  is 11.3.11's formula to the last unit with its divisions rounded
  down.

**The alpha modes** on a source whose alpha is 0x80: replaced by a
field of 0x40 gives 0x40, multiplied by a field of 0x80 gives 0x40
(0x80 x 0x80 / 255), and the three colour channels are untouched in
both.

**An output in a narrower format is packed as table 59 says**: a fill
of ARGB8888 0xFFFF 8000 written as RGB565 landed as 0xFC00.

**The configuration error is real and it costs nothing.** A destination
address one byte off its 32-bit format, written through the resource's
own registers past the driver's refusal, raised CEIF at the START and
transferred nothing: TCIF stayed clear. That is why the driver's
refusals sit in front of the flag.

**The colour tables, both ways.** Sixteen entries written by the CPU
read back word for word while the engine is idle, and an L8 rectangle
converted through them is exact entry for entry. Then 256 entries
loaded by the ENGINE out of memory: the load started, CTCIF was raised
when it finished, and all 256 words read back exactly.

**THE BANDWIDTH, against a display controller on the same memory.** The
accelerator filling 256x256 ARGB8888 rectangles in the external memory
as fast as it can, while the display controller fetches a frame buffer
out of the same device at 65.3 Hz:

| displayed | fetched | fill | fill rate | total over the bus |
|-----------|---------|------|-----------|--------------------|
| one layer, 16 bpp | 10.0 MB/s | 39..40 px/us | 156..160 MB/s | 166..170 MB/s |
| one layer, 32 bpp | 20.1 MB/s | 36..37 px/us | 144..148 MB/s | 164..168 MB/s |
| two layers, 32 bpp | 40.1 MB/s | 30 px/us | 120 MB/s | 160 MB/s |

The total is the same in all three within six per cent: **this board's
memory delivers some 160 MB/s and the accelerator gets what the display
has not taken**, with the display served first and never starved (the
FIFO underrun flag stayed clear in every configuration). The ranges are
the spread over several runs, a pixel a microsecond, which is the
refresh landing differently against the fill. Those are this board's
numbers - the device's rate, a 16-bit bus at 90 MHz and its refresh -
and not the family's.

**The dead time throttles.** With 32 AHB cycles between accesses the
fill drops from 30 to **25 pixels a microsecond** under the heaviest
display load.

**A rectangle a frame keeps up.** The suite's last letter repaints a
32x32 block and a 240x40 strip once a frame for three seconds: 196
steps over 196 frames, with no underrun.

**THE TRAP, and it is the compiler's and not the silicon's.** A buffer
this engine writes must be `volatile` to the CPU. The accelerator's
destination address reaches it as a plain integer that nothing in the
program dereferences, so a compiler is entitled to answer a read out of
its own last store: a fill whose result printed as 0x0000 and compared
equal to 0xFC00 in the same function is what said so. Every buffer the
suite hands the engine is declared volatile for that reason.

## Not covered yet

Driver gaps:

- **Nothing of chapter 11 is missing.** Every register, every field and
  every mode has a verb; what is not measured is below.

Implemented but not bench-verified:

- **The watermark interrupt** - LWR and TWIF are in the driver and
  nothing arms them: the event is a progress report on a transfer long
  enough to be worth watching, and every transfer the suite makes ends
  in microseconds. A full-screen conversion with the interrupt armed
  would measure it.
- **Suspend and abort** - both verbs exist and neither is exercised
  against a running transfer, for the same reason: the transfers here
  are too short to interrupt reliably. A fill of the whole external
  memory, suspended halfway and resumed, would measure both, and would
  also measure 11.3.12's promise that an abort leaves TCIF clear.
- **The transfer-complete, transfer-error and CLUT-access-error
  interrupts** - the enables, the flags and the ISR body are all in the
  driver, and the suite waits on the START bit instead of taking a
  vector. A transfer whose completion posts an event to the kernel is
  what would measure the first; an address the bus matrix faults on the
  second; a CPU read of a colour table while the engine uses it the
  third.
- **The 4-bit and alpha-only formats (L4, A4, A8)** - they compile,
  their two refusals (an odd pixel count and an odd line offset) are
  exercised, and no transfer is made in them. What would measure them
  is a stencil font, which is what they are for.
- **RGB888 in and out** - three bytes a pixel, the one format whose
  line length is not a power of two; expressible, refused where the
  chapter refuses it, and never transferred.
- **The background converter's own colour table** - `load_clut(source,
  false)` and `clut_entry(index, colour, false)` reach it and the suite
  blends two DIRECT sources. Two indexed sources blended through two
  different tables is what would measure it.
- **A colour table in the RGB888 format (CCM = 1)** - three bytes an
  entry instead of four; the field is written and the suite loads
  ARGB8888 tables only.
- **The parts with the accelerator other than the one on this desk** -
  the family fixture compiles the whole driver on each of them; the
  32F469IDISCOVERY carries one, and the suite that exercises it is the
  display suite, bound to the DISC1's panel and its memory.
