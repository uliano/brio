# Stratum: off-chip devices (`devices/`)

A directory under `brio/` named for neither a silicon nor a peripheral
design but for a POSITION: what sits OFF the chip and is reached over a
link the chip provides. A display controller behind a four-wire serial
interface, an SPI touch controller, an I2C sensor - each of them is a
part with its own registers and its own command vocabulary, and none of
them knows which family drives it. So the code that speaks to one has no
business in a family's directory, and the code that drives the LINK has
no business knowing what is at the other end.

It sits above `util/` and beside the target strata, and it may include
`util/` and `gfx/` but never a family: a driver here names a link by its
CONCEPT, and the board file is where a family's SPI host meets it.

## What the first inhabitants are

The command tier of [../design/gfx.md](../design/gfx.md) - a panel that
holds its own pixels behind a link, spoken to in MIPI's Display Command
Set. That page holds the WHY; what lives here is the four layers it
names, of which this stratum carries two:

| Layer | Where it lives |
|-------|----------------|
| The VOCABULARY - the commands, the address mode's bits, the pixel format, the packers | `devices/dcs.hpp`, target-free and controller-free |
| A CONTROLLER - what one silicon makes of those words | `devices/ili9481.hpp`, a traits type of constexpr facts |
| The LINK - a command with its parameters, a write of pixel bytes, a read of a run | a concept, realized over a family's own bus |
| The DRIVER - the vocabulary over a traits type over a link | this stratum, with the rotation map and the surface shapes |

## Document map

| Document | What it covers |
|----------|----------------|
| [dcs.md](dcs.md) | The DCS vocabulary: the codes, MADCTL's bits and what they mean to a WALK, COLMOD's two fields, the window packer, the three pixel packers, and the read framings a link has to know about |
| [ili9481.md](ili9481.md) | The ILI9481: its frame memory and the walk its address mode selects, the three read framings of its four-wire serial interface, its clock limits against the bench's own numbers, the wake sequence, and the module facts that are a board's and not the controller's |

## How a driver here is tested

Twice, against two different things, and neither substitutes for the
other.

**The host simulator** answers "does the driver do what we believe the
part does". `brio/host/sim_dcs_panel.hpp` is a panel made of RAM - a
core whose seam is the DCS transaction, with a framing adapter per link
in front of it - and `test/test_dcs` and `test/test_ili9481` are its
conformance suites, run by `ctest --preset host` with no hardware. Every
expectation in them is a number the bench measured, so the suite is the
bench's own list transcribed: the framing cases prove the adapter, the
memory cases prove the core.

**The bench** answers "is what we believe true". A driver and a
simulator written from one reading of a command table agree on the same
mistake, and no host test reaches that; only silicon does. So the
authority runs one way: a bench finding corrects the simulator, never
the other way round ([../design/gfx.md](../design/gfx.md), "The three
planes of truth"). What makes the bench plane cheap here is the panel's
own MEMORY READ - the oracle verb - which lets a drawing be judged pixel
for pixel with no eye on the glass.
