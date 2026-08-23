// USERROW family smoke TU. The User Row is the same 32 memory-mapped
// bytes on every DA/DB package; board_id() must instantiate everywhere.
#include "avrdx/userrow.hpp"

void userrow_common() {
    (void)brio::board_id();
}
