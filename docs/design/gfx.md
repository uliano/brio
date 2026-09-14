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
- **Formatted text composes with the existing print vocabulary** through
  a sink, rather than opening a second path to the screen. A program
  that prints a measurement to a serial line and to a panel writes it
  once.

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
- **The command tier and its panel simulators.** They are built after
  the memory-mapped shape has taken every geometry past the reference
  renderer, because the differential oracle needs one side it already
  trusts.
- **An asynchronous transfer vocabulary.** Not deferred - not
  applicable. Writing to a memory-mapped surface is a store, and the
  panel is scanned by something the program never waits for, so
  run-to-completion holds by construction and there is nothing to await.
  A transfer long enough to break a cooperative dispatch is the command
  tier's problem and arrives with it. What remains here is a budget
  question rather than a design one: a full-screen fill is a long step,
  to be measured and divided if a program cannot afford it.
- **Proportional fonts** - the fixed cell is load-bearing, above.
- **A widget and screen vocabulary** - what an interface is made of,
  as opposed to what it is drawn with. It is born with the interface
  that needs it, over these primitives and without changing them.
