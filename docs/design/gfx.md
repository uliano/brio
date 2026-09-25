# Graphics

`brio/gfx/`. Drawing onto a surface whose pixels may live in the
program's own memory, inside a panel, or nowhere the program can reach -
and the contract that keeps one library correct on all three.

## Why the contract comes before the primitives

The library is built against a simulated surface on the host before any
panel, because a simulation can MODEL the constraints a panel imposes
while a panel can be made to model nothing, and because implementing the
abstractions where they are cheap is what reveals how large the work
really is.

That order has a trap in it, and the contract is the answer to the trap.
A framebuffer in the program's memory is the easy case: everything is
readable, everything is instant, nothing costs a bus transaction. An API
shaped by that case fits no panel that answers no questions, and no test
running on that case would ever notice. So the surface contract, the
conventions below and the colour model are settled first, and the easy
case is then implemented UNDER them rather than allowed to define them.

The conventions in particular are free now and expensive later. Each one
is a silent one-pixel error once primitives exist, and each has to be
answered identically by the reference renderer that judges them.

## Three kinds of surface

The kinds are told apart by where the truth about a pixel lives.

- **Write-only** - nowhere the program can reach. The panel holds the
  pixels and will not say what it holds. Erasing means drawing again in
  the background colour.
- **Read-write** - inside the panel, and it answers. Slow enough that a
  program which can avoid asking should.
- **Memory-mapped** - in the program's own memory, scanned out by
  something the program does not wait for.

They nest as refining concepts: write-only is the base that every
surface satisfies, the others add to it. A panel with a shadow buffer in
RAM is memory-mapped as far as the library is concerned, because the
shadow is where the truth is.

The doctrine that makes the nesting worth anything: **the library draws
through the base and nothing else**, whatever the surface underneath can
do. The extra power of a richer surface is the application's to use, not
the library's.

## The surface contract

The base asks for two verbs, at the granularity panels are good at: a
rectangle filled with one colour, and a window fed a run of pixels. That
is what a panel does natively - address a window, then stream into it.

A single pixel is the degenerate rectangle, not the foundation. A
library built upward from set-a-pixel crawls on any real panel, because
every pixel then costs its own window command; the memory-mapped case
hides this completely, which is exactly why the direction is settled
here and not discovered later.

The readable refinement adds one verb, reading a pixel back. **No
primitive in the library may use it.** That is not a discipline anyone
has to remember: the base concept has no such verb, so a primitive that
reaches for it fails to compile, and the differential oracle below would
catch it even if it did.

Everything else is a free function over the base - clear, a single
pixel, horizontal and vertical runs, lines, rectangles, round
rectangles, circles, the filled forms of each, and text. They work on
every surface without a device implementing anything beyond the two
verbs, and axis-aligned lines route to the run verb, because interface
work is mostly axis-aligned and that path is the one panels are fast at.

## Conventions

- **A rectangle is an origin and an extent, never two corners.** Whether
  a far corner is inside or outside is the commonest off-by-one in
  drawing code. An extent has no such question, and a zero extent means
  nothing with no special case anywhere.
- **A line carries both of its endpoints.** The natural reading, and the
  one a renderer derived from the definition of a segment also produces.
- **Every primitive clips to its surface, silently.** On a
  microcontroller a pixel written outside the buffer corrupts whatever
  lies next to it. Clipping is also the correct behaviour and not merely
  the safe one: an interface that trims a shape at the edge of the
  screen is right, one that refuses to draw it is not.
- **A viewport is a view, not a mode.** Translating and clipping to a
  sub-rectangle is a wrapper around a surface - composable, carrying no
  state down in the base, costing nothing where it is not used. Elements
  of an interface get their sandbox that way, rather than through a clip
  rectangle every primitive has to consult.
- **Drawing state lives above the base.** A pen holds a current point, a
  colour and a background, and forwards to the stateless verbs. The base
  stays explicit, so primitives are order-independent and a golden image
  cannot depend on state nobody can see.
- **A ring and a disc of the same radius are different sets, and
  deliberately.** An outline holds the pixel NEAREST the ideal curve at
  each step - a rounding - while a filled shape holds every pixel whose
  centre lies INSIDE it - a truncation. Measured, the ring falls partly
  outside the disc and does not cover all of the disc's edge, at every
  radius; no adjustment of one radius aligns them, because the two
  discretizations differ in kind. Each is right for its own job: at a
  radius of nine the ring's apex is five pixels wide and reads as round,
  where the disc's edge there is a single pixel and would read as a
  point. So an outline is not a filled shape's border: a filled shape
  with a border of another colour is TWO FILLED SHAPES, the larger drawn
  first - which is also the only form no-compositing allows.
- **A coordinate stays within 4096 of the origin, and so does a
  surface's own size.** A panel a microcontroller drives is at most some
  hundreds of pixels a side, so this leaves room to place a shape
  several screens outside one and let the clipping deal with it. The
  number is not arbitrary: the segment's error term is tested at twice
  its value and reaches THREE TIMES the segment's span - measured, not
  reasoned, and a good deal worse than the algebra suggests - so the
  domain is exactly what lets that stepping stay sixteen bits wide.
  At 4096 its widest intermediate is 24574, a quarter of the type still
  spare. A surface's size is checked against the domain where it is
  declared; a coordinate a caller computes cannot be, which is why the
  bound is generous enough that reaching it means something else has
  already gone wrong - the classic producer of a distant coordinate is
  not a large panel but a long list scrolled by a pixel offset.
- **One bit per pixel is packed row-major, most significant bit
  leftmost, each row a whole number of bytes.** This is the layout of a
  framebuffer in memory, and it is deliberately not any panel's native
  order: a panel that stores columns of pages converts in its own
  driver, where the conversion belongs and where the bench can judge it.

## Colour

The pixel type is a template parameter - the element type is the beat,
the rule the transfer engines already follow. Two depths carry the
weight: one bit, and eight bits read either as a grey level or as an
index into a table of 256 colours.

A palette travels with the surface, because nothing can display an
indexed pixel without one. The one-bit case carries two colours for the
same reason, and gains something by it: a panel is white on blue or
black on green, so an observer that knows the pair shows what the
hardware will show instead of an idealized black and white.

## Text

Text is monospaced, and that is a consequence rather than a limitation.
A fixed cell is what lets a field erase itself by being rewritten, which
is the only erasing a write-only surface has.

- **Glyphs are opaque.** A cell is drawn foreground and background
  together, so writing a character replaces whatever was there without
  reading it. This is what makes text work on a panel that answers no
  questions.
- **A font is a type.** The cell is then a constant, field arithmetic
  resolves at compile time, and the linker drops the fonts a program
  does not name - fonts are the largest data in a small program, so this
  is not a micro-optimization. Scaling by whole numbers is an adapter
  over one font; a drawn font per size is the other road; both satisfy
  the same contract and the application chooses between them.
- **A cell includes its advance** (a six-by-eight cell for a five-by-
  seven glyph), so consecutive cells tile with no gaps and the opaque
  background covers the whole run.
- **A field pads to a fixed number of cells**, which turns "the previous
  value is gone" from a recommendation into a verb.
A field is what an interface updates, so the field verb is the one that
matters; drawing a bare string is what it is built from.

## No compositing

A shape does not blend with what lies under it and does not ask what
lies under it. This closes alpha, z-order, the damage of occluded
regions and every read-modify-write in one decision - and the
read-modify-write is one the write-only base could not have offered
anyway.

What follows in practice: an interface is laid out so that its elements
do not overlap, erasing is drawing again in the background colour, and
decorations drawn once when a screen is entered are never crossed by the
fields that update inside it.

## What it costs

"Pays nothing it is not asked for" is a claim, so here is what it
actually weighs. Measured as flash, compiled for size, over a
one-bit-per-pixel panel of 128 by 64, each row adding to the one above
it:

| what a program uses | CH32V00x | AVR DA/DB |
|---|---|---|
| clear and a filled rectangle | 386 | 628 |
| plus the outline and the segment | 582 | 818 |
| plus the circle, the disc and the two rounded rectangles | 1604 | 2492 |
| plus text and a five-by-seven font | 2833 | 3970 |

Two things worth reading off it. The round shapes cost MORE than the
whole of text apart from its glyph table - an integer square root and
four functions are not free, and a program that draws only frames and
fields never pays for them, which is what the free-function layering is
for. And the glyph table is 672 bytes of that last row, so on the
smallest part brio touches - sixteen kilobytes - a complete graphical
program pays about a sixth of its flash, of which a quarter is the
alphabet.

### The one place that still computes wider

The segment is the exception to the rule above: its stepping keeps a
32-bit error term. That is not an oversight, and the numbers say why.

The error doubles - the algorithm tests twice the error against each
axis - and, measured rather than reasoned, it reaches THREE TIMES the
segment's span, not once. So keeping it in sixteen bits means a
coordinate no larger than about 5400, and a shape may legitimately be
placed far outside a surface: the classic producer of distant
coordinates is not a large panel but a long list scrolled by a pixel
offset, where an item's position grows with the list and leans on
clipping to disappear.

Two ways out, when it is worth taking one. Declare a coordinate domain -
4096 is the round number, leaving a quarter of the type spare - which
the surface side can enforce with a static assertion on its dimensions
and the coordinate side only in a debug build. Or CLIP THE SEGMENT
FIRST, which bounds the span by the surface instead of by the type and
so needs no domain at all, the same move that keeps the rectangle clip
narrow; the cost is that clipping a segment correctly means adjusting
its error term and not merely moving its endpoints, or the pixels shift
by one - which the exhaustive comparison against the reference would
catch at once.

Neither is urgent: the wide arithmetic here runs once per pixel of a
segment and not once per primitive, and a segment lying mostly outside
the surface - the only case that wastes much - is rare in an interface.

### What an update costs, and what found it

The other half of "pays nothing it is not asked for" is what a PANEL
pays, which is not the same question: on a framebuffer a call is a store,
and behind a bus it is a window command and a burst of bytes. A surface
that forwards and counts (`brio/gfx/counting.hpp`) is how that stops
being a claim - the same idea as a flash simulation's wear counters.

Measured on a front panel of two framed setpoints: a full repaint is 227
windows, changing one digit is 8, a digit change that carries is 16, and
moving a highlight is 16. That is the write-only economy working - a cell
rewritten opaque needs no erase and no read-back, so a change touches
only what changed, and the difference between 8 and 227 is the
difference between an interface that answers and one that crawls.

The first use of that counter also found something about THIS library:
of a repaint's 227 windows, 107 are single-pixel rectangles, nearly all
of them the ARCS of two rounded frames - because a circle's mirrors are
drawn a pixel at a time. In memory that is free. Behind a bus it is a
hundred window commands for one frame, which is precisely what the base
verbs were chosen to avoid. Coalescing an arc's shallow runs is the
answer, and it is not built: the counter is what makes the case for it,
and the case should be made with a measurement of the fix and not with
an assumption.

## The three planes of truth

An oracle is worth exactly its independence from the thing it judges.
Three questions here, three different judges, and none of them
substitutes for another.

**Are the primitives right?** A reference renderer answers - derived
from the definition of each shape rather than from an algorithm,
deliberately slow, obviously correct, and free to be as wasteful as it
likes because it never leaves the host. Where the two disagree, the
finding is usually that the SPECIFICATION is missing rather than that
either one is wrong: where the centre of a circle falls between pixels,
how thick an arc is where it runs shallow, whether the seam between two
octants is drawn twice. Writing the reference is what forces those to be
decided, one per primitive and early; after the decision, agreement
means something. What no reference can judge is whether the result is
legible - that is a golden image looked at once by a human and then
frozen.

**Is the pipeline right?** The differential oracle answers: the same
scene drawn through a memory-mapped surface and through a panel's
driver, compared pixel by pixel. The primitives are the same code on
both sides, and that is the point rather than a flaw - what differs is
everything downstream of them, the window arithmetic, an inclusive end,
the orientation, the packing, the clipping, the splitting into runs. It
follows that this oracle cannot catch a wrong primitive and must not be
trusted to.

**Does the driver match the silicon?** Only the bench answers. A driver
and a panel simulator written from one reading of a command table agree
on the same mistake, and no host test reaches that. A bench finding
corrects the simulator, never the other way round.

## The command tier

A panel that holds its own pixels behind a link - a serial interface, a
parallel bus, a MIPI DSI host - is the write-only or the read-write
surface above, and every such panel this project meets speaks the same
command vocabulary, MIPI's DCS (the window, the memory write and read,
the address mode, the pixel format), over a link of its own. Three
decisions keep the tier from being written once per panel.

**Two axes, four layers.** The VOCABULARY is one target-free file:
the DCS commands, the address mode's bits, the pixel format codes, the
window and pixel packers, the read formats. A CONTROLLER is a traits
type over it - what the ILI9481 does with a window's walk, a read's
dummy byte, the BGR bit, its clock limits - and so is every other DCS
controller; the small OLEDs have a vocabulary of their own on the same
shape. The LINK is a concept, `DcsLink`: a command with its parameters,
a write of pixel bytes, a read of a run, each either synchronous or
answered later - the four-wire serial link over a family's SPI request,
the 8080 parallel bus where a store is the transfer, the DSI host's
packets, I2C for the OLEDs. A panel driver is the vocabulary over a
traits type over a link, and it lives in the devices stratum
(`brio/devices/`), the stratum for what sits off the chip, of which it
is the first inhabitant. The MODULE's facts - which way its glass is
mounted, its colour order, whether it wants the inversion - are a board
matter, handed to the driver as the pins are.

**Three surface shapes, and which one a program gets.** Memory-mapped:
the framebuffer in RAM scanned by a controller, the LTDC or the DSI host
in video mode. DIRECT: the base verbs as synchronous link transactions -
the natural shape where a store is the transfer, and as polled requests
over a serial bus the shape of a bring-up, of a suite, and of a program
that owns its bus alone. TILED: the primitives draw into a rectangle of
RAM, a strip or a dirty rectangle, at the memory-mapped speed, and a
display active object streams the tile to the panel as one link
transaction, the bus asynchronous and the kernel free meanwhile; the
DSI's adapted command mode is this shape with a tile the size of the
screen, made by the hardware. What is NOT built is the per-run
asynchronous surface, where every primitive's run becomes a queued
request: the primitives produce runs faster than any queue drains, and
every buffer would be on loan until its reply.

**One oracle verb.** A read of a run on the link - the panel's memory
read - is how the bench plane judges every command panel: the same test
over the serial link, over DSI and over the parallel bus, the
memory-mapped shape comparing its own RAM instead. The rotation map -
the logical surface over the glass's frame, the walk bits each rotation
needs - is the driver's, so a read comes back in logical coordinates
and a test is written once for every orientation.

The panel simulator on the host has the same two axes. A CORE whose
seam is the DCS transaction - the memory and the walk, written from the
bench's findings and corrected by them, never the other way round - and
a FRAMING ADAPTER per link in front of it: bytes with select and D/C
for the serial link, packets for DSI, stores for the parallel bus. A
simulated host at the request level drives it, and so can a captured
trace. The bench suite of a panel and the conformance suite of its
simulator are one list of cases: the framing ones prove the adapter, the
memory ones prove the core.

## What is deliberately absent

- **Transparent text.** Not deferred but absent: a glyph that leaves its
  background alone has to know what is already there, and the base has
  no verb with which to ask.
- **Antialiasing.** Possible later, and text is the reason it is
  possible at all - a cell always writes its own background, so a blend
  has a known ground to blend against and can be computed when the glyph
  is built. It cannot exist at one bit per pixel for want of levels, and
  on an indexed surface only where the palette carries the ramp, which
  is the application's decision and not the library's.
- **Dirty rectangles.** They pay for themselves where a flush exists,
  and in the memory-mapped shape there is none. The surface is shaped so
  that a decorator can accumulate them later without a primitive
  changing.
- **An asynchronous transfer vocabulary in the primitives.** Not
  deferred - not applicable. Writing to a memory-mapped surface is a
  store, and the panel is scanned by something the program never waits
  for, so run-to-completion holds by construction and there is nothing
  to await. On a command panel the tiled shape of the command tier
  keeps it that way: the primitives write RAM, the transfer is the
  display object's. What remains is a budget question rather than a
  design one: a full-screen fill is a long step, to be measured and
  divided if a program cannot afford it.
- **A stroke wider than one pixel.** Not an oversight and not hard to
  compute - it is hard to SPECIFY, which is why it waits. A thin segment
  has one obviously right answer (the nearest pixel at each step); a
  thick one has to say where its ends stop (cut square across the
  segment, extended past it, or rounded) and what happens where two
  strokes meet (mitred, bevelled, rounded), and each of those is a
  decision no reference renderer can settle on its own. The likely
  specification, when it comes, is the CAPSULE - every pixel within half
  the width of the segment - which the reference computes from the
  definition exactly as it computes the shapes here, and which is not
  the same set as several thin segments drawn side by side. A thick
  AXIS-ALIGNED stroke needs none of this and already exists: it is a
  filled rectangle. Nothing in the design forecloses the general case;
  the pen carries a width when a drawing asks for one, and the
  primitives gain overloads rather than changing.
- **A sink that makes formatted text reach a panel.** The intent is
  settled - a program that prints a measurement to a serial line and to
  a screen should write it once, through the print vocabulary that
  already exists, rather than through a second path of the screen's own
  - and so is the mechanism, since a byte sink is a static contract and
  a surface is an object: the surface arrives as a reference template
  parameter, the way a journal reaches its panic reporter. What is NOT
  settled is what such a sink does at the end of a line and at the edge
  of its surface, and those are the whole of its behaviour. It is built
  when that is decided, not before.
- **Proportional fonts** - the fixed cell is load-bearing, above.
- **A widget and screen vocabulary** - what an interface is made of,
  as opposed to what it is drawn with. It is born with the interface
  that needs it, over these primitives and without changing them.
