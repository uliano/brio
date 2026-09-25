# The DCS vocabulary

Documents of record: none of its own. The MIPI Display Command Set
specification is NOT on this project's desk, so nothing here is cited by
a clause of it; every code below is the one a controller's own command
table lists under the same name, and the authority for what a code MEANS
is the controller's document beside this one
([ili9481.md](ili9481.md)). Driver: `devices/dcs.hpp`. Suite:
`test/test_dcs` (host, no hardware). The design that put this file where
it is, and the four layers it belongs to, are
[../design/gfx.md](../design/gfx.md)'s "The command tier".

## What the vocabulary is

**One file, no state, no link.** A panel that keeps its own pixels
behind a serial interface, a parallel bus or a DSI host is addressed in
the same words whatever the link and whatever the controller: open a
window with a column range and a page range, write pixels into it, read
them back, set the address mode, set the pixel format. Those words are
here, as constants and as pure arithmetic. Nothing here sends anything:
there is no panel object, no timing and no wait, so the file compiles on
every compiler of the tree and costs an image that does not use a name
exactly nothing.

**The WALK is not here.** MADCTL's three high bits reorder the way a
panel's address counter crosses a window, and what they do to it is a
property of the silicon: the constants are here, their effect is the
controller's document. This is the one place where the division between
this file and a traits type is easy to get wrong, and getting it wrong
is how a vocabulary silently becomes one panel's driver.

**A ROUND TRIP IS EXACT ONLY ON THE BITS THE FORMAT KEEPS.** Eighteen
bits a pixel keep six bits a channel (mask 0xFC on each byte), sixteen
keep five, six and five (0xF8, 0xFC, 0xF8), twenty-four keep everything.
Unpacking puts the kept bits back in the MOST significant bits of each
channel and leaves the rest at zero - it does not replicate the high
bits down, which some libraries do to make a colour look right. So
`unpack(pack(c))` equals `c & mask` and nothing weaker is promised, and
a test that compares a read-back against what was written must mask what
was written the same way.

**A colour is the drawing world's.** 0x00RRGGBB in a `uint32_t`, red in
the most significant byte, which is what `gfx/`'s surfaces carry and
what a panel driver hands down. A packer's `bgr` argument is the
exchange - of the first and third BYTE at eighteen and twenty-four bits,
of the two FIELDS at sixteen, where a channel is not a byte. It exists
for two different reasons that happen to need the same operation: the
address mode's BGR bit, and a module wired with its colour channels
crossed.

**A read's framing is a named thing.** On a link that clocks whole
bytes, a panel's answer is not always byte-aligned with the command that
asked for it: some commands answer one byte late, some one CLOCK late
(the whole stream shifted by a bit), some are not driven at all. Which
of those a command takes is the controller's fact, so `DcsReadFraming`
is the vocabulary's word for it and a traits type is what says which.

## Types and verbs

`Dcs` - the command codes, a scope with a deleted constructor rather
than an enum, because these are bytes on a link and travel beside a
controller's own manufacturer commands: `nop`, `swreset`, the read
registers `rddid` / `rddst` / `rddpm` / `rddmadctl` / `rddcolmod` /
`rddim` / `rddsm` / `rddsdr`, the power pair `slpin` / `slpout`, the
mode commands `ptlon` / `noron` / `idmon` / `idmoff`, the inversion pair
`invon` / `invoff`, `gamset`, the display pair `dispon` / `dispoff`, the
window pair `caset` / `paset`, the memory verbs `ramwr` / `ramrd` /
`ramwr_continue` / `ramrd_continue`, `ptlar`, the tearing pair `teon` /
`teoff`, `madctl`, `colmod`, and the brightness group `wrdisbv` /
`rddisbv` / `wrctrld` / `rdctrld`.

`DcsAddressMode` - MADCTL's bits under the standard's names, with what
each means to brio beside it:

| Name | Bit | What it is |
|------|-----|------------|
| `page_order` | B7, 0x80 | the walk along the PAGES reversed |
| `column_order` | B6, 0x40 | the walk along the COLUMNS reversed |
| `exchange` | B5, 0x20 | the counter steps the PAGES first |
| `line_order` | B4, 0x10 | line address order - a display bit |
| `bgr` | B3, 0x08 | a pixel's channels in the other order |
| `horizontal_flip` | B1, 0x02 | a display bit |
| `vertical_flip` | B0, 0x01 | a display bit |
| `walk_bits` | 0xE0 | the three that are the walk, for a mask |

`DcsPixelFormat` - COLMOD's two three-bit fields, the command
interface's format in bits 2:0 (DBI) and the display interface's in 6:4
(DPI), with the codes `bpp3`, `bpp16`, `bpp18`, `bpp24` and the two
masks. `dcs_colmod(dbi, dpi)` builds the byte, `dcs_colmod_dbi` and
`dcs_colmod_dpi` take it apart, and `dcs_colmod_18bpp` is 0x66 - named
because it is the reset value of the controller documented beside this.

The window: `dcs_window(start, end)` gives the four parameter bytes of
CASET or PASET, each address big-endian, THE END INCLUSIVE;
`dcs_window_range(span)` is its inverse and gives a `DcsWindowRange`.

The pixel packers, all constexpr, each with a `bgr` flag that defaults
to false: `dcs_rgb666_pack` / `dcs_rgb666_unpack` (three bytes),
`dcs_rgb565_pack` / `dcs_rgb565_unpack` (two bytes, most significant
first on the wire), `dcs_rgb888_pack` / `dcs_rgb888_unpack` (three
bytes).

`DcsReadFraming` - `undriven`, `aligned`, `dummy_clock`, `dummy_byte`.

## How to use it

A window and a memory write, as a driver sends them:

```cpp
const auto columns = brio::dcs_window(x0, x1);       // CASET's four bytes
const auto pages = brio::dcs_window(y0, y1);         // PASET's four bytes
link.command(brio::Dcs::caset, columns);
link.command(brio::Dcs::paset, pages);
link.write(brio::Dcs::ramwr, pixel_bytes);   // the memory verb, not the command one
```

`link` there is a `DcsLink` ([dcs_link.md](dcs_link.md)); this file
contributes the bytes and knows nothing of how they travel.

A run of pixels packed for an eighteen-bit interface:

```cpp
for (uint16_t i = 0; i < n; ++i) {
    const auto p = brio::dcs_rgb666_pack(run[i]);
    bytes[i * 3] = p[0];
    bytes[i * 3 + 1] = p[1];
    bytes[i * 3 + 2] = p[2];
}
```

A read-back compared against what was written - masked, because the
format kept six bits of each channel:

```cpp
const uint32_t back = brio::dcs_rgb666_unpack(std::span<const uint8_t, 3>(got));
CHECK(back == (written & 0x00FCFCFCu));
```

The address mode as a program states it, rather than as a magic number:

```cpp
using A = brio::DcsAddressMode;
const uint8_t code = A::exchange | A::page_order;    // the pages first, downward
link.command(brio::Dcs::madctl, std::span<const uint8_t, 1>(&code, 1));
```

## Bench findings

None are this file's. It holds no behaviour, so `test/test_dcs` is what
judges it: the packers' round trips over a swept and a sampled colour
space in all three formats and both channel orders, the window packer
and its inverse over every one of the 65536 addresses, and every
constant against its number. Where the packers meet a panel - and what
a panel does with a window - is [ili9481.md](ili9481.md)'s bench
findings.

## Not covered yet

Driver gaps:

- **The commands with parameters this file only names**: `gamset`,
  `ptlar`, `teon` and the brightness group have codes here and no
  packers, because what their parameters mean differs between
  controllers far more than a window does - each is born with the driver
  that sends it.
- **The other pixel formats** - three bits a pixel, and the twelve-bit
  packing some controllers offer. Declined until a panel here uses one:
  a packer with no caller is a packer nothing checks.
- **A colour type other than 0x00RRGGBB** (a palette index, one bit a
  pixel). The surfaces that carry those exist in `gfx/`; what is missing
  is a panel that takes them, and the conversion belongs to that panel's
  driver and not here.

Implemented, not bench-verified: nothing. Every function of this file is
pure arithmetic exercised by `test/test_dcs`, and what it produces
reaches silicon through the controller document beside this one.
