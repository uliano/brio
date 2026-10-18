// USERROW family smoke TU. The User Row is the same memory-mapped row
// on every package - 32 bytes on the AVR DA/DB, 64 on the AVR EA, at
// 0x1080 on both; board_id() must instantiate everywhere.
#include "avrxt/userrow.hpp"

void userrow_common() {
    (void)brio::board_id();
}
