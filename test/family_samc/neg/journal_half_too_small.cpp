// mcu: samc21e18a samc21g18a samc21j18a
// One half of the RWWEE attic is 512 bytes = eight program units, and a
// 32-byte value costs one of them. Twenty ids plus the entry being
// written plus the panic reserve is twenty-two: the reserve save() is
// supposed to leave behind could never be there. Must not compile.

#include "samc/nvm_flash.hpp"
#include "util/nv_journal.hpp"

using namespace brio;

NvJournal<RwweeJournalZone, 20, 32, 2> journal;
