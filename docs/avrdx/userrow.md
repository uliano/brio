# USERROW - the board identity label (AVR DA/DB)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B 8.7,
errata DS80000915F (no USERROW items). Driver: `avrdx/userrow.hpp`.
Every `test_avr_*` suite prints the label in its banner.

## What the silicon does

The User Row is 32 bytes of nonvolatile memory, mapped in data space
at `USERROW` (0x1080) and read by the CPU like ordinary memory. Two
properties make it an identity store:

- **A chip erase does not touch it** (DS40002247B 8.7): whatever is
  written there survives every reflash of the part.
- **UPDI can write it even on a locked part** (end-production data is
  its stated purpose); reading it back over UPDI requires the part to
  be unlocked.

The CPU could also write it through the NVMCTRL flash path; brio
deliberately does not drive that path - a label is provisioning, not a
run-time act, and the programmer already owns the write.

Related but different: SIGROW (DS40002247B 8.6) carries a factory
serial number, unique per chip and read-only. It identifies the die;
USERROW carries the name WE chose for the board, which is what a human
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
#include "avrdx/userrow.hpp"

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

The bench manifest (`tools/bench_boards.py`) records the label each
desk position is expected to carry; the suites' banners let the human
(or a script) compare.

## Bench findings

- Labels `brio-a` / `brio-b` written on the two bench boards over
  UPDI read back correctly from firmware and survive reflashing the
  application (the banner still names the board after a new
  `-U flash:w`).
