// mcu: samc21e18a samc21g18a samc21j18a
// A 200-byte value costs four program units of the eight a half has, so
// two ids plus the one being written plus the reserve do not fit. The
// geometry assertion is the whole point of declaring max_payload.
// Must not compile.

#include "samc/nvm_flash.hpp"
#include "util/nv_journal.hpp"

using namespace brio;

NvJournal<RwweeJournalZone, 2, 200, 2> journal;
