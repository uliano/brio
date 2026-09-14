# The simulator, and what it shares with a viewer

A program running on the host can put pixels on a screen and take input
from a knob, through a second process that shows the one and drives the
other. This is the contract between those two processes: what is shared,
how they find each other, and the rules that keep it portable.

Drivers: `brio/host/shared_segment.hpp` (the mechanism),
`brio/host/sim_display.hpp` (pixels out), `brio/host/sim_panel.hpp`
(input in), `brio/host/sim_input.hpp` (the devices those inputs drive).
The viewer is `cli/gfxview.py`, run as `brio view <name>`. Reference
test suites: `test_gfx` and `test_quadrature`.

## Why two processes

A single process would have to reconcile two event loops - the kernel's
`run()` and a GUI toolkit's - and the reconciliation always costs the
thing worth having. Driving the kernel from a GUI timer means `run()` is
never called and the idle path is never exercised, which is precisely
what a simulator should be stressing; putting the kernel on its own
thread means `HostPlatform`'s critical section has to become a real lock
and stops being the honest single-threaded thing the tests rely on.

Separating them buys three more things. The boundary PHYSICALLY enforces
the write-only discipline of the drawing library: in that tier the
pixels live in the viewer and the program cannot read them even by
mistake. The two halves need not share a compiler, so the machine
running the viewer and the machine building the program may differ
freely. And the viewer, knowing nothing of drawing, can never quietly
become part of the oracle that judges the drawing.

## One mechanism, both directions

Everything shared is a named region of memory the PROGRAM owns: the
framebuffer it writes and the viewer reads, and the panel the viewer
writes and it reads.

Input was going to be a socket, and a socket was the wrong answer. An
input is a small SNAPSHOT OF STATE and not a stream of gestures, and a
shared region gives for free everything a datagram protocol would have
had to specify: there is nothing to frame and nothing to reassemble,
latest-wins is not a drain rule but the only thing that can happen,
nobody decides which side binds and which connects, and "what if no
viewer is attached" does not arise - the program owns the region, so it
reads as nothing-pressed until someone writes. The one thing a socket
would have added is a notification, and the program does not want one:
it polls in its idle path, because that is what a pad is for and because
nothing pushes into the kernel on silicon either.

**The name is the contract, never a path.** Linux exposes these objects
under `/dev/shm` and macOS exposes them nowhere in the filesystem, so a
viewer attaches with `shm_open` under the same name and the path is a
debugging convenience on one platform only. Two rules come from the same
place and are obeyed everywhere so that what works here works there: a
name is at most 31 characters whole, and a region is SIZED ONCE at
creation, because a second `ftruncate` fails on macOS - so a change of
geometry is a new region and never a resize.

**The program is the owner, in both directions.** A viewer comes and
goes; the program does not. That removes every question about lifetime,
and it is why the input region needs no protocol for absence.

**The boot id is load-bearing**, and the reason is not obvious. After a
region is unlinked and remade under the same name, a viewer's existing
mapping still points at the OLD object, which stays alive as long as it
is referenced - so nothing written inside that object could ever tell
the viewer that anything had changed. Every region therefore carries an
id drawn afresh at creation; a viewer re-opens by name every so often,
compares it, and remaps when it differs. That is also what lets a viewer
stay open across rebuilds and follow a different program at a different
size.

## Pixels out

Name `/brio-gfx-<name>`. All fields little-endian; the pixels begin at a
fixed offset of 1024 bytes whatever the header grows to inside it.

| field | width | meaning |
|---|---|---|
| magic | 4 | `BRGX` |
| version | 2 | of this layout |
| header_bytes | 2 | where the pixels start |
| boot_id | 8 | drawn afresh at every creation |
| width, height | 2 each | pixels |
| stride | 2 | bytes per row |
| format | 1 | 1 = one bit per pixel, row-major, most significant bit leftmost; 8 = one byte per pixel |
| buffers, front | 1 each | 1 and 0 today |
| frame | 4 | bumped by `publish()` |
| palette_used | 2 | how many entries the format reads |
| palette | 256 x 3 | red, green, blue |

The publisher is NOT a surface: it owns the region and hands out the
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

Name `/brio-in-<name>`, 512 bytes.

| field | width | written by | meaning |
|---|---|---|---|
| magic | 4 | program | `BRIP` |
| version | 2 | program | of this layout |
| header_bytes | 2 | program | the whole size |
| boot_id | 8 | program | drawn afresh at every creation |
| buttons, shafts | 1 each | program | slots that mean anything |
| seq | 4 | viewer | bumped when it changes anything |
| pressed | 8 | viewer | one byte a contact, non-zero for ACTIVE |
| shaft | 4 x 4 | viewer | absolute quadrature counts, signed |
| button_name | 8 x 16 | program | what each contact IS |
| shaft_name | 4 x 16 | program | what each shaft IS |

**Two writers, and they never overlap.** The viewer owns what the world
is DOING and the program owns what the panel IS, so there is nothing to
arbitrate. The names are the program's because WHICH CONTACT IS WHICH IS
A FACT ABOUT THE PANEL, and facts about the panel belong in the board
file that describes it - a viewer showing a grid of "button 3" and
"shaft 1" is unusable past two controls.

**A contact is a level and a shaft is an ABSOLUTE count**, both for the
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

A shaft is a knob: **the wheel over it turns it**, one quadrature count
a notch, and it needs no selecting first - the wheel goes to the widget
under the pointer, which is Qt's own behaviour and is also how a hand
reaches for a knob on a real panel. Whichever knob will turn is shown
highlighted, so it is never in doubt. **The middle button presses the
knob's own switch**, with the right button accepted as well so that a
machine without a middle button can still work one.

The controls are laid out as a grid below the screen, labelled with the
names the program gave them. Their ARRANGEMENT is the viewer's and not
the program's: reproducing a panel's real geometry is a great deal of
machinery and buys none of the verification this exists for.

There is no menu and no keyboard control of the widgets. Almost anything
a menu would offer - reset, pause, restart - is the PROGRAM's business
rather than the viewer's, and a viewer commanding a program would need a
third channel, which would reopen every question the snapshot design
just closed: what a lost command means, how a command is told from a
state. The rule that keeps this simple is worth keeping: **what travels
is the state of the world, and nothing else.**

## Not covered yet

Driver gaps:

- **The viewer's input half.** `brio view` shows pixels and does not yet
  write the panel: the widgets a person would press and turn are the GUI
  work, and the contract they will write into is what this page settles.
  The program's side is complete and tested against a writer standing in
  for a viewer.
- **An interactive program to join them to.** The host project builds
  test entries only, so the publisher, the panel and the decoder are each
  proven alone and have nowhere yet to meet. This is the open question of
  whether the host gains a build project of its own, and it also decides
  where the first example application will live.
- **Double buffering.** The fields are reserved and the mechanism is not
  built, because tearing is what the hardware being modelled does and
  hiding it would be a worse simulation, not a better one.
- **A bouncing contact.** DECLINED, not deferred, and the reason is test
  economy rather than effort: absorbing a bounce is `InputScanner`'s
  property and is proven where it lives, so simulating one here would
  prove the same thing again by a longer road. A press is a clean level.
  What the bench still owes is not behaviour but a NUMBER - whether the
  chosen count of stable samples suits a real contact - and no simulator
  can answer that.
- **Commanding the program from the viewer.** No channel carries
  anything but the state of the world, by decision, and a command is an
  event rather than a state. If one is ever wanted it is a separate
  mechanism with a reason of its own, not a field borrowed here.
