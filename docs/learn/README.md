# Learning brio

The route into brio for an embedded programmer who knows C and
microcontrollers and may know little C++ and nothing about active
objects. It has two parts: the ideas, read in order, and the running of
the example, by whichever of two routes suits you.

The ideas, in order; each page assumes the previous:

| Page | What you get from it |
|---|---|
| [From the main loop to active objects](introduction.md) | What problem brio solves and how its program runs: events, queues, active objects, the one scheduling rule, priority, state, time, and what the model costs |
| [The first program, read in stages](first-program.md) | The maintained example `blink`, two active objects and an LED, read mechanism by mechanism in its real source; one complete event path from the interrupt to the LED |
| [The C++ in blink](cpp-notation.md) | A companion to the first program: the notation the example uses, explained through its own lines (aliases, templates, static members, variants, lambdas, `match`); read it from the links in the tutorial, or straight through afterwards |

Running `blink` on the ST Nucleo-G0B1RE, one route or the other; both
cover the same steps, and the first program links to them for readers
who want the LED blinking before they read:

| Route | What you get from it |
|---|---|
| [Build, flash and debug from the command line](command-line.md) | The terminal route: prerequisites, configure, build, flash, observe, gdb, the host tests, with what has been verified stated once at the top |
| [Working in VS Code](vscode.md) | The graphical route with the same steps, and for each step whether it is verified, configured, terminal-only or blocked |

The documented board is the ST Nucleo-G0B1RE, chosen because `blink`
builds for it, its page names the LED, and its debugger configuration
exists in the repository. The same application exists for the AVR,
SAM C21, STM32F4 and CH32V00x projects; the route's command-line and
VS Code steps have the same shape there with that family's project
directory, preset and probe, documented in each target's own page
([the documentation map](../README.md)).

## After the route

The learning route stops where the reference begins. What it leads to:

- The design of the kernel, decision by decision, with the exact
  contracts: [docs/design/kernel.md](../design/kernel.md). Read after
  the first program, when its names mean something.
- The services over the kernel: a line reader that posts each received
  line as an event, and `print` ([serial.md](../design/serial.md)),
  shared SPI and I2C buses
  with an arbiter ([spi-bus.md](../design/spi-bus.md),
  [i2c-bus.md](../design/i2c-bus.md)), the power model
  ([power.md](../design/power.md)), storage in flash
  ([nv-heap.md](../design/nv-heap.md), [nv-journal.md](../design/nv-journal.md)).
- Each target's page: toolchain, boards, probe, debugger, and one
  document per peripheral driver with how to use it
  ([the documentation map](../README.md)).
- The headers under `brio/kernel/` and `brio/util/`: the reference for
  every type and function, with its context rules and ownership rules
  in its comment.

## What the route does not have yet

`blink` is the one maintained teaching example. There is no hands-on
example in this route for a button or any other interrupt-driven input,
for a state machine with more than one state, for the serial console,
for a bus transaction with a borrowed buffer, for deep sleep or for two
cores. Each of these is explained conceptually in the pages above or in
the design pages they link to, and exercised by the test suites each
target's documentation names; a teaching example for each is a stated
gap, to be written and compiled on a board before any page describes
it.
