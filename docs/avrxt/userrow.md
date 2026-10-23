# USERROW - the board identity label (AVR DA, DB and EA)

This document is for a programmer who needs a program to say which
physical board it runs on: two identical chips on two identical boards
cannot be told apart by their code, and a USB bridge without a serial
number cannot be told apart by its port. The User Row is a small
nonvolatile row that a chip erase does not touch, and on a brio bench it
holds a short text label given to the board when it was set up.
`board_id()`, the one function of `avrxt/userrow.hpp`, reads that label
and returns it as text; every `test_avr_*` suite prints it in its
banner. Writing the label is provisioning, done once per board over
UPDI with the programmer; the program only reads it.

The smallest use, from "What brio makes of it" below:

```cpp
#include "avrxt/userrow.hpp"

auto board = brio::board_id();          // "brio-a", or empty
if (board.empty()) board = "?";
```

Line by line: the include brings in `board_id()`; the call returns the
label as a `std::string_view` (a pointer and a length, no copy), here
`"brio-a"`, or an empty view when the row holds no valid label; the last
line replaces an empty view with a placeholder before the label is
printed.

What decides the result even there: the label must have been written
over UPDI first ("What brio makes of it" gives the avrdude line); a row
that is erased, unterminated or holds a non-printable byte comes back
as the empty view, which the code checks byte by byte. Nothing checks
that the label is the one the board is expected to carry: the bench
manifest records the expected labels, and the comparison is the
reader's or a script's. `board_id()` waits for nothing, masks no
interrupt and needs no clock or peripheral brought up first, so it may
be called from any context, before `SysClock::init()` included; the
view it returns points into the memory-mapped row itself. On the AVR
DA/DB the CPU can also erase and write the row, and an erase takes the
label with the rest ([nvm.md](nvm.md), "Writing the User Row").

## Documents of record

AVR128DB28/32/48/64 data sheet DS40002247B 8.7,
errata DS80000915F (no USERROW items); for the AVR64EA28/32/48
DS40002443A 8.2, 8.6, 8.7 and 11.3.1.3 (64 bytes of RWW flash at the
same address; the CPU's write path is the flash's page buffer,
declined there with flash self-programming - [nvm.md](nvm.md)), errata
DS80001048D (no USERROW items). Driver: `avrxt/userrow.hpp`.
Every `test_avr_*` suite prints the label in its banner; the read on
the AVR EA is judged by `test_avr_nvm`'s letter a (a letter is one
command key of a suite's console and the group of verdicts it runs;
`z` runs the suite's single-board set).

## What the silicon does

The User Row is 32 bytes (64 on the AVR EA) of nonvolatile memory, mapped in data space
at `USERROW` (0x1080) and read by the CPU like ordinary memory. Two
properties make it an identity store:

- **A chip erase does not touch it** (DS40002247B 8.7): whatever is
  written there survives every reflash of the part.
- **UPDI can write it even on a locked part** (end-production data is
  its stated purpose); reading it back over UPDI requires the part to
  be unlocked.

On the AVR DA/DB the CPU can also write it through the NVMCTRL Flash
commands - the row is erased and written as Flash, one byte at a time,
with the erase taking all 32 bytes at once ([nvm.md](nvm.md),
`Nvm::userrow_write` and `Nvm::userrow_erase`). On the AVR EA the row
is one 64-byte page written through the flash's page buffer, and the
CPU's write is declined with the part's flash self-programming: the
same verbs refuse there at compile time, with the reason, and
`Nvm::userrow_read` reads the row. `board_id()` stays a read on both:
a label is provisioning, not a run-time act, and the programmer
already owns the write.

Related but different: SIGROW (DS40002247B 8.6) carries a factory
serial number, unique per chip and read-only. It identifies the die;
USERROW carries the name given to the board, which is what a human
at the bench wants to read.

## What brio makes of it

Identical chips on identical boards are indistinguishable once the USB
cabling is in doubt (the bench CH340 bridges carry no USB serial), so
each board's USERROW holds a label: printable ASCII from byte 0,
NUL-terminated inside the row. An erased row (0xFF) is an unlabeled
board.

`board_id()` returns the label as a `std::string_view`, validating
instead of trusting: an erased, unterminated or non-printable row
comes back as the empty view, never as garbage on the console.

```cpp
#include "avrxt/userrow.hpp"

auto board = brio::board_id();          // "brio-a", or empty
if (board.empty()) board = "?";
print(serial, "suite banner (board ", board, ", ...)", crlf);
```

Provisioning a new board, once, over UPDI (ASCII bytes plus the
terminating NUL; the rest of the row stays 0xFF):

```bash
avrdude -c atmelice_updi -p avr128db48 -P usb:<probe-serial> \
        -U userrow:w:0x62,0x72,0x69,0x6f,0x2d,0x61,0x00:m   # "brio-a"
avrdude ... -U userrow:r:-:h                                # read back
```

The bench manifest (`cli/bench/bench_boards.py`) records the label each
desk position is expected to carry; the suites' banners let the human
(or a script) compare.

## Bench findings

- Labels `brio-a` / `brio-b` written on the two bench boards over
  UPDI read back correctly from firmware and survive reflashing the
  application (the banner still names the board after a new
  `-U flash:w`).
- On the AVR64EA48 the label written over UPDI (`-p avr64ea48`, the
  same `-U userrow:w:...:m`) reads back through `board_id()` and
  `Nvm::userrow_read()` alike (`test_avr_nvm` a).
