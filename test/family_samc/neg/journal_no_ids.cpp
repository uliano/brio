// mcu: samc21e18a samc21g18a samc21j18a
// A journal that can hold no id at all is not a journal. Must not compile.

#include "samc/nvm_flash.hpp"
#include "util/nv_journal.hpp"

using namespace brio;

NvJournal<RwweeJournalZone, 0, 32, 2> journal;
