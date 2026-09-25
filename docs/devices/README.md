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
names - all four, a family owing the tier nothing but the bus its own
`SpiHost` already offers:

| Layer | Where it lives |
|-------|----------------|
| The VOCABULARY - the commands, the address mode's bits, the pixel format, the packers | `devices/dcs.hpp`, target-free and controller-free |
| A CONTROLLER - what one silicon makes of those words | `devices/ili9481.hpp`, a traits type of constexpr facts |
| The LINK - a command with its parameters, a write of pixel bytes, a read of a run | `devices/dcs_link.hpp`: the `DcsLink` concept, and the four-wire serial realization over ANY stratum's `SpiHost` |
| The DRIVER - the vocabulary over a traits type over a link | `devices/dcs_panel.hpp`: the rotation map, the DIRECT surface shape and the oracle verb |

## Document map

| Document | What it covers |
|----------|----------------|
| [dcs.md](dcs.md) | The DCS vocabulary: the codes, MADCTL's bits and what they mean to a WALK, COLMOD's two fields, the window packer, the three pixel packers, and the read framings a link has to know about |
| [ili9481.md](ili9481.md) | The ILI9481: its frame memory and the walk its address mode selects, the three read framings of its four-wire serial interface, its clock limits against the bench's own numbers, the wake sequence, and the module facts that are a board's and not the controller's |
| [dcs_link.md](dcs_link.md) | The link: the `DcsLink` concept a panel driver is written over - three synchronous verbs, a read that comes back RAW - and `DcsSerialLink<Host>`, the four-wire serial realization whose configuration is TWO PROTOTYPE REQUESTS of the application's own bus, so that one file serves every stratum's `SpiHost` verbatim |
| [dcs_panel.md](dcs_panel.md) | The driver: the logical surface over the glass - the rotation map (a window plus the walk bits that make a logical row one contiguous walk of the address counter), the module's mirror and colour order answered where each belongs, the two Surface verbs as synchronous link transactions, and the memory read back in logical coordinates as the oracle every orientation is judged by |

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
memory cases prove the core. One level up,
`brio/host/sim_spi_host.hpp` is a SPI host made of RAM whose seam is the
Request, so the code a driver actually calls - the link, and above it
the driver - is judged against that panel with no silicon anywhere:
`test/test_dcs_link` and `test/test_dcs_panel` are those suites, the
second stacking the whole tier and judging it twice, against the
simulated memory and against the reference renderer of
`brio/host/gfx_reference.hpp`. Beside them, every family fixture
directory carries a `devices_link.cpp` and a `devices_panel.cpp`, so
`brio check` proves that what names no family compiles for every part
of every one.

**The bench** answers "is what we believe true". A driver and a
simulator written from one reading of a command table agree on the same
mistake, and no host test reaches that; only silicon does. So the
authority runs one way: a bench finding corrects the simulator, never
the other way round ([../design/gfx.md](../design/gfx.md), "The three
planes of truth"). What makes the bench plane cheap here is the panel's
own MEMORY READ - the oracle verb - which lets a drawing be judged pixel
for pixel with no eye on the glass.
