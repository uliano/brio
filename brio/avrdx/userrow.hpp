// ============================================================================
//  userrow.hpp - the User Row read as the board's identity label.
//
//  USERROW (DS40002247B 8.7) is a 32-byte nonvolatile section, memory-
//  mapped in data space, that a chip erase does NOT touch: whatever is
//  written there survives every reflash of the part. brio uses it to
//  name the physical BOARD - identical chips on identical boards are
//  otherwise indistinguishable once the USB cabling is in doubt.
//
//  The convention: byte 0 starts a printable-ASCII label, NUL-
//  terminated inside the row ("brio-a"). An erased row (0xFF) is an
//  unlabeled board. The label is provisioning, not a run-time act: it
//  is written once per board over UPDI
//  (avrdude -U userrow:w:0x62,...,0x00:m) and this header only READS
//  it. The write path exists - the row is erased and written with the
//  FLASH commands, byte by byte, the erase taking all 32 bytes at once
//  (avrdx/nvm.hpp: Nvm::userrow_write, userrow_write_block,
//  userrow_erase) - it is simply not something an identity label
//  should be reached through.
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
inline std::string_view board_id() {
    const char *row = reinterpret_cast<const char *>(&USERROW);
    for (uint8_t n = 0; n < sizeof(USERROW_t); ++n) {
        const char c = row[n];
        if (c == '\0') return {row, n};
        if (c < 0x20 || c > 0x7e) return {};
    }
    return {};  // 32 bytes without a terminator: not a label
}

}  // namespace brio
