# The simulator, and what it shares with a viewer

This page is for a programmer who writes a host program with a screen
and a front panel (a brio program built for the native target by the
`host/` project, [README.md](README.md#running-the-suites)) and wants to
watch it draw and work its controls with a mouse, and for whoever
changes the viewer that does the watching. The simulator is two
processes. The program draws into a framebuffer and describes its
controls; the viewer, `brio view <name>`, shows the picture and writes
back what the mouse does to the controls. The program reads that
snapshot in its idle path and hands it to simulated contacts and a
simulated encoder (`brio/host/sim_input.hpp`), which the program's
input scanner and quadrature decoder read exactly as they would read
pins on a board. Everything shared lives in two regions of POSIX shared
memory, each called a segment here: a block of memory with a name,
created by the program and opened by name by the viewer.

The smallest use, abridged from the host program that draws the front
panel of a bench power supply, a display of 256 by 128 one-byte pixels
with one button and one knob:

```cpp
using Screen    = brio::SimDisplay<brio::Indexed8, 256, 128>;
using SelectKey = brio::SimButton<0>;
using PushKey   = brio::SimButton<1>;
using Knob      = brio::SimEncoder<0>;

brio::SimPanel* world = nullptr;

struct SimPlatform : brio::HostPlatform {
    static void idle() {
        ticks = elapsed_ms();                    // the program's own clock reading
        if (world != nullptr) {
            SelectKey::set(world->pressed(0));
            PushKey::set(world->pressed(1));
            Knob::set_shaft(world->shaft(0));
        }
        timespec nap{0, 500000};                 // half a millisecond
        nanosleep(&nap, nullptr);
    }
};

int main() {
    Screen display("supply");                    // creates /brio-gfx-supply
    brio::SimPanel panel("supply");              // creates /brio-in-supply
    world = &panel;
    panel.name_button(0, "V / A");
    panel.name_button(1, "Digit");
    panel.name_shaft(0, "Adjust", 1, 4);         // switch is contact 1, 4 counts a detent
    display.set_palette(0, 0x10, 0x14, 0x1C);

    Screen::Surface surface = display.surface();
    brio::fill_rect(surface, 0, 0, 256, 128, 0);
    display.publish();
    // ... then the kernel's loop, which calls SimPlatform::idle()
}
```

The excerpt leaves out its includes - `gfx/draw.hpp`,
`host/platform.hpp`, `host/sim_display.hpp`, `host/sim_input.hpp`,
`host/sim_panel.hpp`, `<time.h>` for the clock behind an elapsed-
milliseconds helper - and the helper itself. The program's source lives
in `host/src/apps/<program>.cpp`, where the `host/` project discovers
it; it is built with `cd host && cmake --preset apps && cmake --build
--preset apps --target <program>` and run as
`build-cmake/host-apps/<program>` (build, run and the segments a run
leaves behind: verified on Linux), and watched from a second terminal
with `brio view <name>` (the viewer needs Python with PySide6, and says
so when it is missing). The viewer's window: configured, not watched -
what it does, here and below, is read from its source, `cli/gfxview.py`,
and checked by driving the viewer offscreen, never in an interactive
session. macOS: unavailable on this desk, so what the page says of it
is the headers' statement, unchecked.

Line by line: `Screen` is the display type, its pixel format and size
fixed at compile time; `SimButton<n>` and `SimEncoder<n>` are a
simulated contact and a simulated encoder, each its own type with
static state, the number only telling two of them apart; `SimPlatform`
is the host platform (`HostPlatform`, the kernel's view of the
machine) with its `idle()` replaced - the kernel calls `idle()` when
every queue is empty, and here it is where time advances (`ticks` is
`HostPlatform`'s clock variable, in milliseconds, set here from a
function of the program's own that reads the system's monotonic clock),
where the viewer's snapshot is copied into the simulated contacts, and
where a nap of half a millisecond keeps an idle program from spinning a
processor core. In `main`, constructing `display` and
`panel` creates the two segments under the one name `supply`;
`name_button` and `name_shaft` say what the controls are, and a
control exists for the viewer only once it is named; the shaft's two
further arguments say which contact is the switch under its knob and
how many quadrature counts make one detent (one click of the knob);
`set_palette` gives the colour index 0 stands for; `surface()` is an
ordinary framebuffer over the segment's pixels, which the drawing
library draws into like any other; and `publish()` tells the viewer
the picture changed. The kernel's loop is the program's own and is not
shown: `brio::Tenuto` over `SimPlatform`, with an input scanner
(`util/input_scanner.hpp`) reading `SelectKey` and `PushKey` and a
quadrature decoder (`util/quadrature.hpp`) reading `Knob::A` and
`Knob::B`, the encoder's two contacts, passed to it as two pins would
be.

What decides correctness even there:

- **The program creates both segments; the viewer only opens them.**
  They may start in either order. A viewer started first shows a dark
  grey screen titled "waiting for" the name and looks again every
  quarter of a second.
- **Name the controls and set the palette right after construction,
  before anything else.** The viewer reads the palette, the names, the
  switches and the detents once, when it attaches; a later change
  reaches it only when a viewer attaches again (a restarted viewer reads
  the names and the palette as they are then). Nothing marks a segment
  ready, so a viewer already waiting could attach in the moment between
  a constructor and the naming that follows it, and show the default
  palette or no controls - or inside the display's constructor itself,
  after the magic and the header size are written and before the width
  and height, and stay on a zero-size image until the boot id changes;
  each window is that moment against a look every quarter of a second,
  narrow but not closed, and restarting the program clears both.
- **Call `publish()` after a drawing.** This viewer repaints when the
  frame counter moves (it looks every 16 ms), so pixels written without
  a `publish()` may stay off the screen.
- **The panel is read only where the program copies it.** The snapshot
  becomes the contacts' state in `idle()`, so the scanner and the
  decoder see the mouse only as often as the kernel runs out of work.
- **A name is short.** The whole name with its prefix is at most 31
  characters (`/brio-gfx-` takes 10 of them, `/brio-in-` 9), so a name
  given to both the display and the panel, as in the smallest use, is at
  most 21 characters; a longer one throws `std::runtime_error` from the
  constructor, and the viewer refuses it too.
- **One running program per name.** A program's constructor removes
  whatever segment holds its name before creating its own, and its
  destructor removes the name again on the way out, so a second program
  under the same name orphans the first and may later remove the
  second's segment.
- **A stale segment is harmless and is replaced.** A program stopped by
  Ctrl-C, by another signal whose default action kills it, or by a crash
  runs no destructor, and its two segments stay behind under their names
  (on Linux they are visible in `/dev/shm`, and deleting them there
  removes them by hand; that both survive SIGINT and SIGTERM, that the
  next run replaces them and that a normal exit removes them was checked
  on Linux). Meanwhile a viewer goes
  on showing the last picture, and its controls write into memory no
  program reads. The next run under that name removes the stale
  segments before making its own, the viewer finds a new boot id (an
  identifier drawn afresh at each creation, "One mechanism, both
  directions" below) within a quarter of a second, and it follows. A
  program that exits normally removes its segments; within a quarter of
  a second the viewer's screen turns dark grey and its controls go, and
  it attaches again when a program under that name next runs.
- **With no viewer attached, nothing changes for the program.** The
  panel's segment is zeroed at creation, so every contact reads as
  released and every shaft at count 0 until a viewer writes.

The rest of the page is the contract behind those rules: why there are
two processes, the one mechanism both directions share, the layout of
each segment, and what the viewer's controls do.

## Documents of record

Drivers: `brio/host/shared_segment.hpp` (the mechanism),
`brio/host/sim_display.hpp` (pixels out), `brio/host/sim_panel.hpp`
(input in), `brio/host/sim_input.hpp` (the devices those inputs drive).
The viewer is `cli/gfxview.py`, run as `brio view <name>`. Reference
test suites: `test_gfx` and `test_quadrature`.

## Why two processes

A single process would have to reconcile two event loops - the kernel's
loop, which serves the active objects' queues and calls `idle()` when
they are empty ([the learning route's
introduction](../learn/introduction.md#the-kernel-a-loop-that-serves-queues)),
and a GUI toolkit's - and the reconciliation always costs the thing
worth having. Driving the kernel from a GUI timer means the kernel's
own loop (`TimeEvents::process()`, `step()`, `idle_if_empty()`, written
out as this program writes it or as `run()`) is never the one in charge
and the idle path is never exercised, which is precisely what a
simulator should be stressing; putting the kernel on its own
thread means `HostPlatform`'s critical section (a counter of how deep
the guard is held, [README.md](README.md#the-host-platform)) has to
become a real lock and stops being the plain single-threaded counter
the tests rely on.

Separating them buys two more things and makes room for a third. For a write-only panel - the
kind that holds its pixels and never gives them back, the surface the
drawing library draws through
([../design/gfx.md](../design/gfx.md#three-kinds-of-surface)) - the
boundary would enforce that discipline physically: a panel modelled
inside the viewer would hold pixels the program cannot read even by
mistake. No such panel is built; the display published today is the
memory-mapped kind, its pixels in the program's own memory, where the
program can read them like any framebuffer ("Pixels out" below). The
two halves need not share a language or a compiler: the viewer is
Python, so only the program needs the gnu++23 compiler the rest of
brio needs. And the viewer, knowing nothing of drawing, can never
quietly become part of the oracle that judges the drawing (the
reference renderer the tests compare the drawing library against,
[README.md](README.md#simulated-devices-and-what-they-are-allowed)).

## One mechanism, both directions

Everything shared is a named region of memory the program owns: the
framebuffer it writes and the viewer reads, and the panel the viewer
writes and it reads.

A socket is the obvious carrier for input, and the wrong one. An
input is a small snapshot of state and not a stream of gestures, and a
shared region gives for free everything a datagram protocol would have
had to specify: there is nothing to frame and nothing to reassemble,
latest-wins is not a drain rule but the only thing that can happen,
nobody decides which side binds and which connects, and "what if no
viewer is attached" does not arise - the program owns the region, so it
reads as nothing-pressed until someone writes. The one thing a socket
would have added is a notification, and the program does not want one:
it polls in its idle path, as firmware polls the pad (the point where a
button's wire meets the chip) the button is wired to; the one thing that
pushes into a kernel on silicon is a post from an interrupt handler, and
this simulator has no interrupt.

**The name is the contract, never a path.** Linux exposes these objects
under `/dev/shm` and macOS exposes them nowhere in the filesystem, so a
viewer attaches with `shm_open` under the same name and the path is a
debugging convenience on one platform only. Two rules come from the same
place and are obeyed everywhere so that what works here works there: a
name is at most 31 characters whole, and a region is sized once at
creation, because a second `ftruncate` fails on macOS - so a change of
geometry is a new region and never a resize. The macOS facts are the
headers' (`host/shared_segment.hpp`), not checked on this desk.

**The program is the owner, in both directions.** A viewer comes and
goes; the program does not. That removes every question about lifetime,
and it is why the input region needs no protocol for absence.

**The boot id is load-bearing**, and the reason is not obvious. After a
region is unlinked and remade under the same name, a viewer's existing
mapping still points at the old object, which stays alive as long as it
is referenced - so nothing written inside that object could ever tell
the viewer that anything had changed. Every region therefore carries an
id drawn afresh at creation; a viewer re-opens by name every so often
(this one every quarter of a second), compares it, and remaps when it
differs. That is also what lets a viewer stay open across rebuilds and
follow a different program at a different size.

## Pixels out

Name `/brio-gfx-<name>`, 1024 bytes of header and then the pixels, one
row after another. All fields in the host's native byte order,
little-endian on x86-64 and arm64 (nothing checks it); the pixels begin
at a fixed offset of 1024 bytes whatever the header grows to inside it.

| field | width | meaning |
|---|---|---|
| magic | 4 | `BRGX` |
| version | 2 | of this layout |
| header_bytes | 2 | where the pixels start: 1024, and the viewer refuses any other value |
| boot_id | 8 | drawn afresh at every creation |
| width, height | 2 each | pixels |
| stride | 2 | bytes per row |
| format | 1 | 1 = one bit per pixel (`Mono`), row-major, most significant bit leftmost; 8 = one byte per pixel (`Indexed8`) |
| buffers, front | 1 each | 1 and 0 today |
| reserved | 3 | zero |
| frame | 4 | bumped by `publish()`; the viewer repaints when it moves |
| palette_used | 2 | how many entries the format reads |
| palette | 256 x 3 | red, green, blue |

The publisher, `SimDisplay`, is not a surface: it owns the region and hands out the
bytes an ordinary `Framebuffer` draws into, so nothing in `brio/gfx/`
learns where its pixels live and the same library code runs against a
plain array in a test.

**The palette travels** because a byte on its own shows nothing. A
one-bit surface carries two colours, which is what lets an observer
render a panel as it really looks - white on blue, black on green -
instead of an idealized black and white.

**Tearing is honest.** With one buffer a viewer can read a row the
program is part way through writing, exactly as a real memory-mapped
panel tears unless something swaps its buffers at the right moment. The
`buffers` and `front` fields exist for the day that stops being
acceptable; adding them later would have changed the layout under every
viewer already written.

## Input in

Name `/brio-in-<name>`, 512 bytes, all fields in the host's native byte
order (little-endian on x86-64 and arm64, nothing checks it). The slots
are fixed, eight contacts and four shafts: a name given to a slot past
them is ignored, and reading one answers released or count 0.

| field | width | written by | meaning |
|---|---|---|---|
| magic | 4 | program | `BRIP` |
| version | 2 | program | of this layout |
| header_bytes | 2 | program | the whole size |
| boot_id | 8 | program | drawn afresh at every creation |
| buttons, shafts | 1 each | program | how many slots the layout has: 8 and 4 |
| reserved | 6 | - | zero |
| seq | 4 | viewer | bumped when it changes anything; a program may watch it, and need not |
| pressed | 8 | viewer | one byte a contact, non-zero for active |
| shaft | 4 x 4 | viewer | absolute quadrature counts, signed |
| button_name | 8 x 16 | program | what each contact is |
| shaft_name | 4 x 16 | program | what each shaft is |
| shaft_switch | 4 x 1 | program | the contact under that knob, or 0xFF for none |
| shaft_detent | 4 x 1 | program | quadrature counts the part gives per detent, 4 until named otherwise |

**Two writers, and they never overlap.** The viewer owns what the world
is doing and the program owns what the panel is, so there is nothing to
arbitrate. The names are the program's because which contact is which
is a fact about the panel, and facts about the panel belong in the
board file that describes it (the part of a program that says what its
hardware is and which pin or simulated contact plays which role) - a
viewer showing a grid of "button 3" and "shaft 1" is unusable past two
controls.

**A control exists when it is named.** The slots are fixed, so naming
one is how a program declares it and an unnamed slot is simply one it
does not use. Two more facts about a knob are declared beside its name,
because a viewer cannot work them out for itself: which contact sits
under it, because a viewer that decided for itself which button belonged
to which knob would be inventing a fact about the panel; and how many
counts make one of its detents, because a notch of a wheel is a click of
a knob and a viewer has to know how far that is.

**A contact is a level and a shaft is an absolute count**, both for the
same reason: a snapshot that is lost costs nothing, because the next one
carries the whole truth. The viewer owns the count; the program mirrors
it.

**The mirror snaps rather than walks**, and that is faithful rather than
lazy. Putting the contacts straight where the shaft now is - instead of
stepping them through everything it passed - is what a decoder sampling
once a tick sees anyway. A shaft that moved three counts between two
samples reads as one count backwards on real hardware too, and a
simulator that walked the pads to hide it would be hiding the one
failure a quadrature decoder has.

**Which contact drives which device is not here either.** A board file
wires `pressed(0)` to its own `SimButton` and `shaft(0)` to its own
`SimEncoder`, exactly as a target's board file wires a pin, and the
program does it in its idle path - which is where the world turns.

## What the viewer's controls do

A contact is a button under the pointer: **left click presses it**, and
releasing the mouse releases it.

A shaft is a knob: **the wheel over it turns it, one detent a notch of
120 wheel units** - a finer wheel's events (a touchpad's) are floored to
whole notches by the viewer, so small steps forward turn nothing and
small steps backward a whole detent, a defect of the viewer's -
and it needs no selecting first - the wheel goes to the widget
under the pointer, which is Qt's own behaviour and is also how a hand
reaches for a knob on a real panel. The control under the pointer is
drawn highlighted, so which knob will turn is never in doubt. **The
middle button presses the knob's own switch**, with the right button
accepted as well so that a machine without a middle button can still
work one.

A notch is not delivered at once. The counts of a detent are queued and
walked, one every 8 milliseconds, the way a shaft passes through every
state on its way round - and this is not decoration. A whole detent
handed over in one step is a turn no hand could make, and a decoder
sampling once a tick sees only where the contacts ended up: at four
counts a detent they are back where they started and nothing is
credited, and at two a state is skipped, which the decoder counts as
lost and, by default, credits nothing for (`test_quadrature`, "a shaft
outrunning the poll degrades in a known way"). The viewer's job here is
to be a hand, and a hand takes time.

The controls are laid out in a row below the screen, the knobs first
and then the contacts, labelled with the names the program gave them,
and a contact that is a knob's own switch is drawn as part of that knob
rather than again on its own. Their arrangement is the viewer's and not
the program's: reproducing a panel's real geometry is a great deal of
machinery and buys none of the verification this exists for.

There is no menu and no keyboard control of the widgets. Almost anything
a menu would offer - reset, pause, restart - is the program's business
rather than the viewer's, and a viewer commanding a program would need a
third channel, which would reopen every question the snapshot design
just closed: what a lost command means, how a command is told from a
state. The rule that keeps this simple is worth keeping: **what travels
is the state of the world, and nothing else.**

## Not covered yet

Driver gaps:

- **Double buffering.** The fields are reserved and the mechanism is not
  built, because tearing is what the hardware being modelled does and
  hiding it would be a worse simulation, not a better one.
- **A bouncing contact.** Declined, not deferred, and the reason is test
  economy rather than effort: absorbing a bounce is `InputScanner`'s
  property and is proven where it lives, so simulating one here would
  prove the same thing again by a longer road. A press is a clean level.
  What the bench still owes is not behaviour but a number - whether the
  chosen count of stable samples suits a real contact - and no simulator
  can answer that.
- **Commanding the program from the viewer.** No channel carries
  anything but the state of the world, by decision, and a command is an
  event rather than a state. If one is ever wanted it is a separate
  mechanism with a reason of its own, not a field borrowed here.

Implemented but not bench-verified:

- **The viewer's window.** Its behaviour on this page is read from
  `cli/gfxview.py` and checked by driving it offscreen with synthetic
  events; an interactive session in front of the window would verify it.
- **macOS.** The 31-character cap, the single `ftruncate` and the
  absence of a filesystem path are the headers' statements; a program
  and a viewer run on a Mac would verify them.
