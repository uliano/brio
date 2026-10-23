// ============================================================================
//  userrow.hpp - the User Row read as the board's identity label.
//
//  What it does for a caller: board_id() returns the label written into
//  the User Row when the board was provisioned, so a program - every
//  test_avr_* suite's banner - can say which physical board it runs on.
//  The smallest use:
//
//      auto board = brio::board_id();      // "brio-a", or empty
//      if (board.empty()) board = "?";
//
//  Who calls it: any context, at any time. It reads the memory-mapped
//  row with plain loads, at most one row's length of them, waits for
//  nothing and masks nothing. The view it returns points into the row
//  itself, not into a copy: it stays valid for the life of the program,
//  and shows other bytes only if the CPU rewrites the row (on the AVR
//  DA/DB, Nvm::userrow_write or Nvm::userrow_erase).
//
//  USERROW (DS40002247B 8.7; DS40002443A 8.2 and 8.6 on the AVR EA) is a
//  nonvolatile section of 32 bytes on the AVR DA/DB and 64 on the AVR EA
//  (sizeof(USERROW_t) either way), memory-mapped in data space at 0x1080,
//  that a chip erase does not touch: whatever is written there survives
//  every reflash of the part. brio uses it to name the physical board -
//  identical chips on identical boards are otherwise indistinguishable
//  once the USB cabling is in doubt.
//
//  The convention: byte 0 starts a printable-ASCII label, NUL-
//  terminated inside the row ("brio-a"). An erased row (0xFF) is an
//  unlabeled board. The label is provisioning, not a run-time act: it
//  is written once per board over UPDI
//  (avrdude -U userrow:w:0x62,...,0x00:m) and this header only reads
//  it. On the AVR DA/DB the write path exists - the row is erased and
//  written with the flash commands, byte by byte, the erase taking all
//  32 bytes at once (avrxt/nvm.hpp: Nvm::userrow_write,
//  userrow_write_block, userrow_erase) - it is not something an
//  identity label should be reached through; on the AVR EA the CPU's
//  USERROW write is declined with its flash self-programming (the row
//  is written through the flash's page buffer, avrxt/nvm.hpp refuses the
//  verbs by name; docs/avrxt/nvm.md), Nvm::userrow_read() reads it, and
//  UPDI writes the row even on a locked part (DS40002443A 8.7).
//
//  board_id() validates instead of trusting: a row that does not follow
//  the convention (erased, no terminator, non-printable bytes) comes
//  back as the empty view, never as garbage on the console.
// ============================================================================

#pragma once

#include <avr/io.h>

#include <string_view>

namespace brio {

/// The board label from USERROW: the NUL-terminated printable-ASCII
/// string at byte 0, or the empty view when the row holds no valid label.
/// Callable from any context; the view points into the mapped row.
inline std::string_view board_id() {
    const char *row = reinterpret_cast<const char *>(&USERROW);
    for (uint8_t n = 0; n < sizeof(USERROW_t); ++n) {
        const char c = row[n];
        if (c == '\0') return {row, n};
        if (c < 0x20 || c > 0x7e) return {};
    }
    return {};  // a whole row without a terminator: not a label
}

}  // namespace brio
